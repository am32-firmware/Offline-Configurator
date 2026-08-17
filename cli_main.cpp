/*
  SerialPortConnector_CLI - headless AM32 ESC configurator used as a protocol
  testbed. It reuses the shared protocol logic in FourWayIF + hexfile so it
  cannot diverge from the GUI on protocol behaviour. Only the serial transport
  glue is CLI-local.

  Commands:
    settings [port]            read + decode the 48-byte EEPROM, save settings.dat
    flash <firmware.hex> [port] flash firmware, preserving/initialising the EEPROM

  Connects through a Betaflight FC via MSP passthrough -> BLHeli 4-way (115200).
*/
#include <QCoreApplication>
#include <QSerialPort>
#include <QIODevice>
#include <QFile>
#include <QElapsedTimer>
#include <cstdint>
#include <cstdio>

#include "fourwayif.h"
#include "hexfile.h"
#include "defaults.h"
#include "sitltransport.h"
#include "BF_ROOTLOADER.h"

static const char *DEFAULT_PORT =
    "/dev/serial/by-id/usb-Betaflight_Betaflight_STM32H743_345D344E3139-if00";
static const char *DEFAULT_HEX =
    "/home/tridge/project/UAV/AM32/AM32/obj/AM32_ARK_G431_CAN_2.20.hex";

// true when talking the direct 19200 1-wire bootloader protocol (SITL or
// a direct single-wire adapter) rather than MSP passthrough -> 4-way
static bool g_direct = false;

// open the transport for a port spec: a "sitl:..." spec uses the UDP
// transport in direct mode, anything else a real QSerialPort. Returns a
// heap QIODevice the caller owns, or nullptr on failure
static QIODevice *openTransport(const QString &port, int baud) {
  if (SitlUdpTransport::isSitlSpec(port)) {
    g_direct = true;
    SitlUdpTransport *t = new SitlUdpTransport(port);
    if (!t->open(QIODevice::ReadWrite)) {
      fprintf(stderr, "failed to open %s: %s\n", qPrintable(port),
              qPrintable(t->errorString()));
      delete t;
      return nullptr;
    }
    return t;
  }
  QSerialPort *sp = new QSerialPort();
  sp->setPortName(port);
  sp->setBaudRate(baud);
  sp->setDataBits(QSerialPort::Data8);
  sp->setParity(QSerialPort::NoParity);
  sp->setStopBits(QSerialPort::OneStop);
  sp->setFlowControl(QSerialPort::NoFlowControl);
  if (!sp->open(QIODevice::ReadWrite)) {
    fprintf(stderr, "failed to open %s: %s\n", qPrintable(port),
            qPrintable(sp->errorString()));
    delete sp;
    return nullptr;
  }
  return sp;
}

// MSP frame builder, identical to Widget::send_mspCommand
static void sendMsp(QIODevice &sp, uint8_t cmd, const QByteArray &payload) {
  QByteArray m;
  m.append((char)0x24);
  m.append((char)0x4d);
  m.append((char)0x3c);
  m.append((char)payload.length());
  m.append((char)cmd);
  if (payload.length() > 0)
    m.append(payload);
  uint8_t cs = 0;
  for (int i = 3; i < m.length(); i++)
    cs ^= (uint8_t)m[i];
  m.append((char)cs);
  sp.write(m);
  sp.waitForBytesWritten(100);
}

// Read a complete response: wait for the first bytes then drain until idle.
static QByteArray blockingRead(QIODevice &sp, int idleMs = 200, int totalMs = 2000) {
  QByteArray buf;
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < totalMs) {
    if (sp.waitForReadyRead(idleMs))
      buf += sp.readAll();
    else if (!buf.isEmpty())
      break;  // got a message, then went idle => complete
  }
  return buf;
}

// One 4-way transaction with retries; returns true on good ACK.
static bool fourWayTxn(QIODevice &sp, FourWayIF &fw, const QByteArray &cmd,
                       QByteArray &payload, int writeWaitMs, int retries) {
  for (int t = 0; t <= retries; t++) {
    fw.ack_required = true;
    sp.write(cmd);
    sp.waitForBytesWritten(writeWaitMs);
    QByteArray resp = blockingRead(sp);
    if (fw.parseFourWayResponse(resp, payload))
      return true;
  }
  return false;
}

/*
  direct 1-wire helpers (SITL bootloader, or a direct single-wire
  adapter): the transport echoes our own transmission, so strip the
  echoed command bytes before parsing the reply.
 */
static BF_ROOTLOADER g_rl;

// send a direct command and read back replyLen bytes after the echo.
// Drains any pending input first (e.g. the self-echo of a preceding
// no-reply command like SET_BUFFER) so the echo accounting stays aligned
static QByteArray directTxn(QIODevice &sp, const QByteArray &cmd, int replyLen,
                            int totalMs = 1000) {
  sp.readAll();
  sp.write(cmd);
  sp.waitForBytesWritten(50);
  QByteArray got;
  QElapsedTimer t;
  t.start();
  const int want = cmd.size() + replyLen;  // echo + reply
  while (t.elapsed() < totalMs && got.size() < want) {
    if (sp.waitForReadyRead(200))
      got += sp.readAll();
  }
  if (got.size() < cmd.size())
    return QByteArray();
  return got.mid(cmd.size());  // strip the echo
}

// set the protocol address, returns true on ACK
static bool directSetAddress(QIODevice &sp, uint16_t address) {
  QByteArray r = directTxn(sp, g_rl.setAddress(address), 1);
  return r.size() >= 1 && (uint8_t)r[r.size() - 1] == 0x30;
}

// read n bytes (1..256) from the current or given address
static QByteArray directRead(QIODevice &sp, int n, int address = -1) {
  if (address >= 0 && !directSetAddress(sp, (uint16_t)address))
    return QByteArray();
  QByteArray r = directTxn(sp, g_rl.readFlash((uint8_t)(n & 0xFF)), n + 3, 2000);
  // reply is n data + 2 CRC + 1 ACK
  if (r.size() < n + 3 || (uint8_t)r[n + 2] != 0x30)
    return QByteArray();
  if (!g_rl.checkCRC(r.left(n + 2), n + 2))
    return QByteArray();
  return r.left(n);
}

// write buf at address (page-erase + program), returns true on ACK
static bool directWrite(QIODevice &sp, uint16_t address, const QByteArray &buf) {
  if (!directSetAddress(sp, address))
    return false;
  // set buffer size (no ack)
  sp.write(g_rl.setBufferSize((uint16_t)buf.size()));
  sp.waitForBytesWritten(50);
  QByteArray r = directTxn(sp, g_rl.sendBuffer(buf), 1, 2000);
  if (r.isEmpty() || (uint8_t)r[r.size() - 1] != 0x30)
    return false;
  QByteArray w = directTxn(sp, g_rl.writeFlash(), 1, 3000);
  return w.size() >= 1 && (uint8_t)w[w.size() - 1] == 0x30;
}

// device-info probe + v3 devinfo struct read over the direct wire
static bool directConnect(QIODevice &sp, FourWayIF &fw) {
  fw.direct = true;
  // 17-byte device-info probe (matches Widget's direct connect)
  QByteArray probe(17, char(0));
  probe[8] = char(13);
  probe[9] = char(66);
  probe[16] = char(0x7d);
  QByteArray info = directTxn(sp, probe, 9, 2000);
  if (info.size() < 9 || !(info[0] == '4' && info[1] == '7' && info[2] == '1')) {
    fprintf(stderr, "no device info from direct bootloader\n");
    return false;
  }
  fw.parseDeviceInfo(info, /*direct=*/true);
  // v3 devinfo struct via the magic address (27 bytes)
  QByteArray block = directRead(sp, 27, ADDRESS_MAGIC_DEVINFO);
  if (block.size() == 27)
    fw.parseDevinfoBlock(block);
  printf("connected (direct): eeprom_address=0x%04x ", fw.eeprom_address);
  if (fw.devinfo_v3.enabled)
    printf("v3 firmware_start=0x%04x address_shift=%u\n",
           fw.devinfo_v3.firmware_start, fw.devinfo_v3.address_shift);
  else
    printf("firmware_start=0x%04x\n", fw.firmware_start);
  return true;
}

static bool openAndConnect(QIODevice &sp, FourWayIF &fw, uint8_t target) {
  if (g_direct)
    return directConnect(sp, fw);

  fw.direct = false;
  fw.passthrough_started = true;

  // enable MSP passthrough -> 4-way (mirror Widget::connectSerial)
  sendMsp(sp, 0x68, QByteArray());
  blockingRead(sp);
  sendMsp(sp, 0xf5, QByteArray());
  blockingRead(sp);

  // 4-way connect (deviceInfo). The first attempts often fail, so retry.
  QByteArray payload;
  bool ok = false;
  for (int t = 0; t < 12 && !ok; t++) {
    fw.ESC_connected = false;
    fourWayTxn(sp, fw, fw.makeFourWayCommand(0x37, target), payload, 200, 0);
    ok = fw.ESC_connected;
  }
  if (!ok) {
    fprintf(stderr, "ESC not connected (no/invalid deviceInfo)\n");
    return false;
  }

  // The 4-way InitFlash reply only carries a 4-byte signature (flash-size
  // code), not the protocol version or firmware start. Read the devinfo
  // struct via the magic address to get those. On older bootloaders this
  // read fails and we keep the flash-size-code defaults.
  QByteArray block;
  // read the full v3 devinfo struct (magic1/2 + 9-byte deviceInfo + 9-byte
  // v3 extension = 27 bytes); older bootloaders just return fewer bytes
  if (fourWayTxn(sp, fw, fw.makeFourWayReadCommand(27, ADDRESS_MAGIC_DEVINFO),
                 block, 300, 2) &&
      fw.parseDevinfoBlock(block)) {
    printf(
        "devinfo via magic read: version=%u firmware_start=0x%04x "
        "address_shift=%u\n",
        fw.bootloader_version,
        fw.devinfo_v3.firmware_start,
        fw.devinfo_v3.address_shift);
  } else {
    printf("devinfo magic read unavailable; using flash-size-code defaults\n");
  }

  printf("connected: bootloader_version=%u eeprom_address=0x%04x ",
         fw.bootloader_version, fw.eeprom_address);
  if (fw.devinfo_v3.enabled) {
    printf("v3 firmware_start=0x%04x address_shift=%u\n",
           fw.devinfo_v3.firmware_start, fw.devinfo_v3.address_shift);
  } else {
    printf("firmware_start=0x%04x divider=%d\n",
           fw.firmware_start, (int)fw.memory_divider_required_four);
  }
  return true;
}

// Read the 48-byte config (filename region + config read in one 80-byte read).
static QByteArray readSettings(QIODevice &sp, FourWayIF &fw) {
  if (g_direct) {
    QByteArray r = directRead(sp, 80, fw.eepromReadAddress());
    if (r.size() >= 80)
      return r.mid(32, 48);
    return QByteArray();
  }
  QByteArray payload;
  for (int t = 0; t < 5; t++) {
    if (fourWayTxn(sp, fw, fw.makeFourWayReadCommand(80, fw.eepromReadAddress()),
                   payload, 300, 0) &&
        payload.size() >= 80) {
      payload.remove(0, 32);  // strip the 32-byte file-name region
      return payload.left(48);
    }
  }
  return QByteArray();
}

static bool writeEeprom(QIODevice &sp, FourWayIF &fw, const QByteArray &buf) {
  if (g_direct)
    return directWrite(sp, fw.eepromWriteAddress(), buf);
  QByteArray payload;
  return fourWayTxn(sp, fw,
                    fw.makeFourWayWriteCommand(buf, buf.size(), fw.eepromWriteAddress()),
                    payload, 500, 3);
}

static void printSettings(const QByteArray &c, const FourWayIF &fw) {
  printf("--- settings ---\n");
  printf(" boot bit          : 0x%02x %s\n", (uint8_t)c[0],
         (uint8_t)c[0] == 0xFF   ? "(blank/bootloader)"
         : (uint8_t)c[0] == 0x01 ? "(valid)"
                                 : "");
  printf(" eeprom version    : %u\n", (uint8_t)c[1]);
  printf(" bootloader version: %u\n", (uint8_t)c[2]);
  printf(" firmware version  : %u.%02u\n", (uint8_t)c[3], (uint8_t)c[4]);
  printf(" reversed/bidir    : %u / %u\n", (uint8_t)c[17], (uint8_t)c[18]);
  printf(" sinusoidal/comp   : %u / %u\n", (uint8_t)c[19], (uint8_t)c[20]);
  printf(" timing advance    : %u\n", (uint8_t)c[23]);
  printf(" pwm freq (kHz)    : %u\n", (uint8_t)c[24]);
  printf(" startup power     : %u\n", (uint8_t)c[25]);
  printf(" motor kv (x40)    : %u\n", (uint8_t)c[26]);
  printf(" motor poles       : %u\n", (uint8_t)c[27]);
  printf(" firmware_start    : 0x%04x (from deviceInfo)\n", fw.firmware_start);
  printf(" raw               : ");
  for (int i = 0; i < c.size(); i++)
    printf("%02x ", (uint8_t)c[i]);
  printf("\n");
}

static int cmdSettings(const QString &port, uint8_t target) {
  QIODevice *dev = openTransport(port, QSerialPort::Baud115200);
  if (!dev)
    return 2;
  QIODevice &sp = *dev;
  FourWayIF fw;
  if (!openAndConnect(sp, fw, target)) {
    dev->close();
    delete dev;
    return 2;
  }
  QByteArray cfg = readSettings(sp, fw);
  if (cfg.size() != 48) {
    fprintf(stderr, "failed to read settings\n");
    return 2;
  }
  printSettings(cfg, fw);
  QFile f("settings.dat");
  if (f.open(QIODevice::WriteOnly)) {
    f.write(cfg);
    f.close();
    printf("saved settings.dat (48 bytes)\n");
  } else {
    fprintf(stderr, "could not write settings.dat\n");
  }
  return 0;
}

static int cmdFlash(const QString &hexPath, const QString &port, uint8_t target) {
  QString err;
  QByteArray image = parseIntelHex(hexPath, &err);
  if (!err.isEmpty() || image.isEmpty()) {
    fprintf(stderr, "hex parse failed (%s): %s\n", qPrintable(hexPath),
            qPrintable(err.isEmpty() ? QString("empty image") : err));
    return 3;
  }
  printf("firmware image: %d bytes from %s\n", image.size(), qPrintable(hexPath));
  // save the exact bytes we flash so a gdb flash-dump can be compared
  {
    QFile fi("flash_image.bin");
    if (fi.open(QIODevice::WriteOnly)) {
      fi.write(image);
      fi.close();
      printf("wrote flash_image.bin (%d bytes)\n", image.size());
    }
  }

  QIODevice *dev = openTransport(port, QSerialPort::Baud115200);
  if (!dev)
    return 2;
  QIODevice &sp = *dev;
  FourWayIF fw;
  if (!openAndConnect(sp, fw, target)) {
    dev->close();
    delete dev;
    return 2;
  }

  // choose the eeprom buffer: preserve a valid existing config, else defaults
  QByteArray cfg = readSettings(sp, fw);
  QByteArray eep;
  if (cfg.size() == 48 && (uint8_t)cfg[0] == 0x01) {
    eep = cfg;
    printf("preserving existing settings\n");
  } else {
    eep = QByteArray((const char *)air_starteeprom, 48);
    printf("using default settings (eeprom was blank)\n");
  }

  // pre-flash safety write: boot bit = 0 (best effort)
  QByteArray e0 = eep;
  e0[0] = 0x00;
  writeEeprom(sp, fw, e0);

  // flash the image in 256-byte chunks; pad the final chunk to a multiple of
  // 8 with 0xFF (STM32 doubleword programming)
  const int chunk = 256;
  const int total = image.size();
  for (int off = 0; off < total; off += chunk) {
    QByteArray c = image.mid(off, chunk);
    while (c.size() % 8 != 0)
      c.append((char)0xFF);
    bool ok;
    if (g_direct) {
      ok = directWrite(sp, fw.firmwareChunkAddress(off), c);
    } else {
      QByteArray payload;
      ok = fourWayTxn(sp, fw,
                      fw.makeFourWayWriteCommand(c, c.size(),
                                                 fw.firmwareChunkAddress(off)),
                      payload, 200, 8);
    }
    if (!ok) {
      fprintf(stderr, "\nFLASH FAILURE at offset 0x%x (ack_type=%d)\n", off,
              fw.ack_type);
      return 4;
    }
    printf("flashed %d/%d\r", off + (int)c.size() > total ? total : off + (int)c.size(),
           total);
    fflush(stdout);
  }
  printf("\n");

  // post-flash write: boot bit = 1
  QByteArray e1 = eep;
  e1[0] = 0x01;
  if (!writeEeprom(sp, fw, e1)) {
    fprintf(stderr, "warning: post-flash eeprom write failed\n");
  }

  // reset/run so the freshly-flashed firmware runs
  if (g_direct) {
    // CMD_RUN: ask the bootloader to jump to the application
    const char run[4] = {0, 0, 0, 0};
    sp.write(QByteArray(run, 4));
    sp.waitForBytesWritten(200);
  } else {
    fw.ack_required = true;
    sp.write(fw.makeFourWayCommand(0x35, target));
    sp.waitForBytesWritten(200);
    blockingRead(sp);
  }

  printf("FLASH SUCCESS\n");
  return 0;
}

// Read N bytes of the EEPROM config region (offset 0 == EEPROM_START).
static QByteArray readEepromN(QIODevice &sp, FourWayIF &fw, int n) {
  if (g_direct) {
    QByteArray r = directRead(sp, n, fw.eepromWriteAddress());
    return r.size() >= n ? r.left(n) : QByteArray();
  }
  QByteArray payload;
  for (int t = 0; t < 5; t++) {
    if (fourWayTxn(sp, fw, fw.makeFourWayReadCommand(n, fw.eepromWriteAddress()),
                   payload, 400, 0) &&
        payload.size() >= n) {
      return payload.left(n);
    }
  }
  return QByteArray();
}

/*
  setparam: read-modify-write a single EEPROM byte. A 4-way EEPROM write
  page-erases the whole EEPROM page, so we must rewrite the full used region
  (config 0-47, startup tune 48-175, CAN params 176-183) rather than just the
  touched byte - otherwise the tune and CAN_NODE/ESC_INDEX would be wiped.
  256 bytes covers the whole used region.
*/
static int cmdSetParam(int offset, int value, const QString &port, uint8_t target) {
  if (offset < 0 || offset > 255 || value < 0 || value > 255) {
    fprintf(stderr, "offset/value out of range (0..255)\n");
    return 1;
  }
  QIODevice *dev = openTransport(port, QSerialPort::Baud115200);
  if (!dev)
    return 2;
  QIODevice &sp = *dev;
  FourWayIF fw;
  if (!openAndConnect(sp, fw, target)) {
    dev->close();
    delete dev;
    return 2;
  }
  QByteArray buf = readEepromN(sp, fw, 256);
  if (buf.size() != 256) {
    fprintf(stderr, "failed to read eeprom region\n");
    return 2;
  }
  printf("setparam: offset %d  %u -> %d\n", offset, (uint8_t)buf[offset], value);
  buf[offset] = (char)value;
  if (!writeEeprom(sp, fw, buf)) {
    fprintf(stderr, "eeprom write failed\n");
    return 4;
  }
  QByteArray rb = readEepromN(sp, fw, 256);
  if (rb.size() == 256 && (uint8_t)rb[offset] == (uint8_t)value) {
    printf("SETPARAM SUCCESS offset=%d value=%d\n", offset, value);
    return 0;
  }
  fprintf(stderr, "SETPARAM VERIFY FAILED\n");
  return 4;
}

// getparam: print one EEPROM byte (works for offsets > 47, unlike settings).
static int cmdGetParam(int offset, const QString &port, uint8_t target) {
  if (offset < 0 || offset > 255) {
    fprintf(stderr, "offset out of range (0..255)\n");
    return 1;
  }
  QIODevice *dev = openTransport(port, QSerialPort::Baud115200);
  if (!dev)
    return 2;
  QIODevice &sp = *dev;
  FourWayIF fw;
  if (!openAndConnect(sp, fw, target)) {
    dev->close();
    delete dev;
    return 2;
  }
  QByteArray buf = readEepromN(sp, fw, 256);
  if (buf.size() != 256) {
    fprintf(stderr, "failed to read eeprom region\n");
    return 2;
  }
  printf("GETPARAM offset=%d value=%u\n", offset, (uint8_t)buf[offset]);
  return 0;
}

/*
  writedefaults: seed a valid default EEPROM config so the bootloader will boot
  the app. jump() refuses unless EEPROM byte 0 == 0x01 (CHECK_EEPROM_BEFORE_JUMP);
  a blank/0xFF EEPROM - e.g. wiped when the bootloader region changes size - never
  boots. Writes the 48-byte default config (air or crawler) into a clean 256-byte
  page. The EEPROM address is resolved by openAndConnect (v3: magic devinfo;
  master: flash-size-code MCU table), so this works on both protocols.

  Note: this writes a clean page, so any existing startup tune (48-175) and CAN
  params (176+) are reset - it is a factory-default seed, not a single-field edit.
*/
static int cmdWriteDefaults(const QString &which, const QString &port, uint8_t target) {
  const uint8_t *def;
  QString name = which.isEmpty() ? QString("air") : which;
  if (name == "air") {
    def = air_starteeprom;
  } else if (name == "crawler") {
    def = crawler_starteeprom;
  } else {
    fprintf(stderr, "unknown defaults set '%s' (use air|crawler)\n",
            qPrintable(name));
    return 1;
  }

  QIODevice *dev = openTransport(port, QSerialPort::Baud115200);
  if (!dev)
    return 2;
  QIODevice &sp = *dev;
  FourWayIF fw;
  if (!openAndConnect(sp, fw, target)) {
    dev->close();
    delete dev;
    return 2;
  }

  QByteArray buf(256, 0x00);
  for (int i = 0; i < 48; i++)
    buf[i] = (char)def[i];

  printf("writedefaults: %s -> eeprom_address=0x%04x (boot byte0=0x%02x)\n",
         qPrintable(name), (unsigned)fw.eepromWriteAddress(), (uint8_t)buf[0]);

  if (!writeEeprom(sp, fw, buf)) {
    fprintf(stderr, "eeprom write failed\n");
    return 4;
  }
  QByteArray rb = readEepromN(sp, fw, 256);
  // the bootloader stamps its own version into byte 2 on every eeprom
  // write, so don't require that byte to match what we sent
  QByteArray expect = buf.left(48);
  if (rb.size() == 256)
    expect[2] = rb[2];
  if (rb.size() == 256 && (uint8_t)rb[0] == 0x01 && rb.left(48) == expect) {
    printf("WRITEDEFAULTS SUCCESS (48 config bytes, boot byte=0x01)\n");
    return 0;
  }
  fprintf(stderr, "WRITEDEFAULTS VERIFY FAILED\n");
  return 4;
}

// Strip --target N / -t N / --target=N from args (anywhere) and return the
// requested 4-way ESC index. Defaults to 0 (motor 1).
static uint8_t extractTargetArg(QStringList &args) {
  uint8_t target = 0;
  for (int i = 1; i < args.size();) {
    const QString a = args[i];
    if ((a == "--target" || a == "-t") && i + 1 < args.size()) {
      target = (uint8_t)args[i + 1].toUInt();
      args.removeAt(i);
      args.removeAt(i);
    } else if (a.startsWith("--target=")) {
      target = (uint8_t)a.mid(QString("--target=").length()).toUInt();
      args.removeAt(i);
    } else {
      i++;
    }
  }
  return target;
}

int main(int argc, char *argv[]) {
  QCoreApplication app(argc, argv);
  QStringList args = app.arguments();
  const uint8_t target = extractTargetArg(args);

  if (args.size() < 2) {
    fprintf(stderr,
            "usage:\n"
            "  %s [--target N] settings [port]\n"
            "  %s [--target N] flash [firmware.hex] [port]\n"
            "  %s [--target N] getparam <offset> [port]\n"
            "  %s [--target N] setparam <offset> <value> [port]\n"
            "  %s [--target N] writedefaults [air|crawler] [port]\n"
            "    --target / -t  4-way ESC index (default 0 = motor 1)\n"
            "    offset/value are EEPROM byte offset (0..255) and value (0..255)\n"
            "defaults: port=%s\n          hex=%s\n",
            qPrintable(args[0]), qPrintable(args[0]), qPrintable(args[0]),
            qPrintable(args[0]), qPrintable(args[0]), DEFAULT_PORT, DEFAULT_HEX);
    return 1;
  }

  const QString cmd = args[1];
  if (cmd == "settings") {
    QString port = args.size() > 2 ? args[2] : QString(DEFAULT_PORT);
    return cmdSettings(port, target);
  } else if (cmd == "flash") {
    QString hex = args.size() > 2 ? args[2] : QString(DEFAULT_HEX);
    QString port = args.size() > 3 ? args[3] : QString(DEFAULT_PORT);
    return cmdFlash(hex, port, target);
  } else if (cmd == "getparam") {
    if (args.size() < 3) {
      fprintf(stderr, "usage: %s [--target N] getparam <offset> [port]\n",
              qPrintable(args[0]));
      return 1;
    }
    int offset = args[2].toInt(nullptr, 0);
    QString port = args.size() > 3 ? args[3] : QString(DEFAULT_PORT);
    return cmdGetParam(offset, port, target);
  } else if (cmd == "setparam") {
    if (args.size() < 4) {
      fprintf(stderr, "usage: %s [--target N] setparam <offset> <value> [port]\n",
              qPrintable(args[0]));
      return 1;
    }
    int offset = args[2].toInt(nullptr, 0);
    int value = args[3].toInt(nullptr, 0);
    QString port = args.size() > 4 ? args[4] : QString(DEFAULT_PORT);
    return cmdSetParam(offset, value, port, target);
  } else if (cmd == "writedefaults") {
    // writedefaults [air|crawler] [port]   (default set = air)
    QString which = "air";
    QString port = QString(DEFAULT_PORT);
    if (args.size() > 2) {
      if (args[2] == "air" || args[2] == "crawler") {
        which = args[2];
        if (args.size() > 3)
          port = args[3];
      } else {
        port = args[2];  // not a set name -> treat as port
      }
    }
    return cmdWriteDefaults(which, port, target);
  }
  fprintf(stderr, "unknown command '%s'\n", qPrintable(cmd));
  return 1;
}
