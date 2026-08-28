/*
  SerialPortConnector_CLI - headless AM32 ESC configurator used as a protocol
  testbed. It reuses the shared protocol logic in FourWayIF + hexfile so it
  cannot diverge from the GUI on protocol behaviour. Only the serial transport
  glue is CLI-local.

  Commands:
    settings [port]            read + decode the 48-byte EEPROM, save settings.dat
    flash <firmware.hex> [port] flash firmware, preserving/initialising the EEPROM
    direct-suite ...            CI exercise of the direct 1-wire protocol
    fourway-suite ...           CI exercise of 4-way discovery and protocol

  Normal commands connect through a Betaflight FC via MSP passthrough ->
  BLHeli 4-way (115200); the direct suite uses the 19200-baud 1-wire protocol.
*/
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QSerialPort>
#include <QThread>
#include <cstdint>
#include <cstdio>

#include "BF_ROOTLOADER.h"
#include "defaults.h"
#include "fourwayif.h"
#include "hexfile.h"

static const char *DEFAULT_PORT =
    "/dev/serial/by-id/usb-Betaflight_Betaflight_STM32H743_345D344E3139-if00";
static const char *DEFAULT_HEX =
    "/home/tridge/project/UAV/AM32/AM32/obj/AM32_ARK_G431_CAN_2.20.hex";

// The suite commands select direct 19200-baud 1-wire operation. Existing
// settings/flash commands retain their 115200-baud 4-way behaviour.
static bool g_direct = false;

// MSP frame builder, identical to Widget::send_mspCommand
static void sendMsp(QSerialPort &sp, uint8_t cmd, const QByteArray &payload) {
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
static QByteArray blockingRead(QSerialPort &sp, int idleMs = 200, int totalMs = 2000) {
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
static bool fourWayTxn(QSerialPort &sp, FourWayIF &fw, const QByteArray &cmd,
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

// Direct adapters echo every transmitted byte before the bootloader reply.
// Return only the reply after validating and removing that echo.
static QByteArray directTxn(QSerialPort &sp, const QByteArray &cmd, int replyLen,
                            int totalMs = 2000) {
  sp.readAll();
  sp.write(cmd);
  // QSerialPort may complete a small PTY write synchronously, in which case
  // waitForBytesWritten() returns false even though the bytes were sent.
  sp.waitForBytesWritten(200);

  QByteArray got;
  QElapsedTimer timer;
  timer.start();
  const int wanted = cmd.size() + replyLen;
  while (timer.elapsed() < totalMs) {
    if (sp.waitForReadyRead(200))
      got += sp.readAll();
    // A prior no-reply command (notably direct RUN) can become readable only
    // after this transaction starts. Locate this command's echo instead of
    // assuming it is the first buffered byte.
    const int echo = got.indexOf(cmd);
    if (echo >= 0 && got.size() >= echo + wanted)
      return got.mid(echo + cmd.size(), replyLen);
  }
  return QByteArray();
}

static BF_ROOTLOADER g_rootloader;

static bool directSetAddress(QSerialPort &sp, uint16_t address) {
  QByteArray reply = directTxn(sp, g_rootloader.setAddress(address), 1);
  return reply.size() == 1 && (uint8_t)reply[0] == 0x30;
}

static QByteArray directRead(QSerialPort &sp, int size, int address = -1) {
  if (address >= 0 && !directSetAddress(sp, (uint16_t)address))
    return QByteArray();
  QByteArray reply = directTxn(
      sp, g_rootloader.readFlash((uint8_t)(size & 0xff)), size + 3);
  if (reply.size() != size + 3 || (uint8_t)reply[size + 2] != 0x30 ||
      !g_rootloader.checkCRC(reply.left(size + 2), size + 2))
    return QByteArray();
  return reply.left(size);
}

static bool directWrite(QSerialPort &sp, uint16_t address,
                        const QByteArray &data) {
  if (!directSetAddress(sp, address))
    return false;
  // SET_BUFFER has an echo but no bootloader reply. Consume its echo before
  // sending the separately CRC-protected payload.
  directTxn(sp, g_rootloader.setBufferSize((uint16_t)data.size()), 0);
  QByteArray reply = directTxn(sp, g_rootloader.sendBuffer(data), 1);
  if (reply.size() != 1 || (uint8_t)reply[0] != 0x30)
    return false;
  reply = directTxn(sp, g_rootloader.writeFlash(), 1, 3000);
  return reply.size() == 1 && (uint8_t)reply[0] == 0x30;
}

static bool directConnect(QSerialPort &sp, FourWayIF &fw) {
  static const char init[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                              0, 0x0d, 'B', 'L', 'H', 'e', 'l', 'i',
                              (char)0xf4, 0x7d};
  QByteArray info = directTxn(sp, QByteArray(init, sizeof(init)), 9);
  if (info.size() != 9 || !fw.parseDeviceInfo(info, /*direct=*/true)) {
    fprintf(stderr, "no valid device info from direct bootloader (%d bytes: %s)\n",
            info.size(), info.toHex(' ').constData());
    return false;
  }
  fw.direct = true;

  QByteArray devinfo = directRead(sp, 27, ADDRESS_MAGIC_DEVINFO);
  if (devinfo.size() == 27)
    fw.parseDevinfoBlock(devinfo);

  if (fw.memory_divider_required_four && !fw.devinfo_v3.enabled) {
    fprintf(stderr, "128k direct targets need a v3 bootloader\n");
    return false;
  }
  printf("connected (direct): bootloader_version=%u eeprom_address=0x%04x ",
         fw.bootloader_version, fw.eeprom_address);
  if (fw.devinfo_v3.enabled)
    printf("v3 firmware_start=0x%04x address_shift=%u\n",
           fw.devinfo_v3.firmware_start, fw.devinfo_v3.address_shift);
  else
    printf("firmware_start=0x%04x\n", fw.firmware_start);
  return true;
}

static bool openSerial(QSerialPort &sp, const QString &port) {
  sp.setPortName(port);
  sp.setBaudRate(g_direct ? QSerialPort::Baud19200 : QSerialPort::Baud115200);
  sp.setDataBits(QSerialPort::Data8);
  sp.setParity(QSerialPort::NoParity);
  sp.setStopBits(QSerialPort::OneStop);
  sp.setFlowControl(QSerialPort::NoFlowControl);
  if (!sp.open(QIODevice::ReadWrite)) {
    fprintf(stderr, "failed to open %s: %s\n", qPrintable(port),
            qPrintable(sp.errorString()));
    return false;
  }
  return true;
}

static bool connectFourWayTarget(QSerialPort &sp, FourWayIF &fw,
                                 uint8_t target) {
  fw.direct = false;
  fw.passthrough_started = true;

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
  if (fw.memory_divider_required_four && !fw.devinfo_v3.enabled) {
    fprintf(stderr, "128k 4-way targets require v3 devinfo\n");
    return false;
  }
  return true;
}

// Enter MSP passthrough and return the ESC count advertised by the FC.
static int startFourWay(QSerialPort &sp) {
  sendMsp(sp, 0x68, QByteArray());
  blockingRead(sp);
  sendMsp(sp, 0xf5, QByteArray());
  QByteArray reply = blockingRead(sp);
  if (reply.size() < 7 || !reply.startsWith("$M>") ||
      (uint8_t)reply[3] < 1 || (uint8_t)reply[4] != 0xf5)
    return 0;
  return (uint8_t)reply[5];
}

static bool openAndConnect(QSerialPort &sp, FourWayIF &fw, const QString &port,
                           uint8_t target) {
  if (!openSerial(sp, port))
    return false;
  if (g_direct)
    return directConnect(sp, fw);
  if (startFourWay(sp) < 1) {
    fprintf(stderr, "4-way interface reported no ESCs\n");
    return false;
  }
  return connectFourWayTarget(sp, fw, target);
}

// Read the 48-byte config (filename region + config read in one 80-byte read).
static QByteArray readSettings(QSerialPort &sp, FourWayIF &fw) {
  if (g_direct) {
    QByteArray payload = directRead(sp, 80, fw.eepromReadAddress());
    return payload.size() >= 80 ? payload.mid(32, 48) : QByteArray();
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

static bool writeEeprom(QSerialPort &sp, FourWayIF &fw, const QByteArray &buf) {
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
  QSerialPort sp;
  FourWayIF fw;
  if (!openAndConnect(sp, fw, port, target))
    return 2;
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

static int flashConnected(QSerialPort &sp, FourWayIF &fw,
                          const QByteArray &image, uint8_t target) {
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
    bool ok = false;
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

  // reset the ESC so the freshly-flashed firmware runs
  if (g_direct) {
    sp.write(QByteArray(4, '\0'));
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

static int cmdFlash(const QString &hexPath, const QString &port, uint8_t target) {
  QString err;
  QByteArray image = parseIntelHex(hexPath, &err);
  if (!err.isEmpty() || image.isEmpty()) {
    fprintf(stderr, "hex parse failed (%s): %s\n", qPrintable(hexPath),
            qPrintable(err.isEmpty() ? QString("empty image") : err));
    return 3;
  }
  printf("firmware image: %d bytes from %s\n", image.size(), qPrintable(hexPath));
  QFile fi("flash_image.bin");
  if (fi.open(QIODevice::WriteOnly))
    fi.write(image);

  QSerialPort sp;
  FourWayIF fw;
  if (!openAndConnect(sp, fw, port, target))
    return 2;
  return flashConnected(sp, fw, image, target);
}

static bool settingsEqualExceptBootloaderVersion(const QByteArray &a,
                                                 const QByteArray &b) {
  if (a.size() != b.size())
    return false;
  for (int i = 0; i < a.size(); i++) {
    if (i != 2 && a[i] != b[i])
      return false;
  }
  return true;
}

// End-to-end CI flow: discover every ESC, mutate and verify settings, flash,
// reset, and reconnect. It deliberately uses the same helpers as the normal
// CLI commands and GUI protocol classes.
static int cmdSuite(const QString &hexPath, const QString &port, bool direct) {
  g_direct = direct;
  QString err;
  QByteArray image = parseIntelHex(hexPath, &err);
  if (!err.isEmpty() || image.isEmpty()) {
    fprintf(stderr, "hex parse failed: %s\n", qPrintable(err));
    return 3;
  }

  QSerialPort sp;
  if (!openSerial(sp, port))
    return 3;
  FourWayIF fw;
  if (direct) {
    if (!directConnect(sp, fw))
      return 5;
  } else {
    const int escCount = startFourWay(sp);
    if (escCount < 1) {
      fprintf(stderr, "4-way interface reported no ESCs\n");
      return 3;
    }
    printf("4-way passthrough, %d ESC(s)\n", escCount);
    for (int target = 0; target < escCount; target++) {
      FourWayIF found;
      if (!connectFourWayTarget(sp, found, (uint8_t)target))
        return 5;
      printf("ESC %d discovered\n", target + 1);
    }
    // Discovery leaves the last channel selected; select motor one for the
    // destructive settings/flash part of the suite.
    if (!connectFourWayTarget(sp, fw, 0))
      return 5;
  }

  QByteArray before = readSettings(sp, fw);
  if (before.size() != 48) {
    fprintf(stderr, "failed to read settings\n");
    return 4;
  }
  QByteArray changed = before;
  changed[30] = (uint8_t)changed[30] == 5 ? 4 : 5;
  if (!writeEeprom(sp, fw, changed)) {
    fprintf(stderr, "settings write failed\n");
    return 4;
  }
  QByteArray readback = readSettings(sp, fw);
  if (!settingsEqualExceptBootloaderVersion(changed, readback)) {
    fprintf(stderr, "settings write/readback mismatch\n");
    return 4;
  }
  printf("settings write+readback ok\n");

  int rc = flashConnected(sp, fw, image, 0);
  if (rc != 0)
    return rc;

  FourWayIF reconnected;
  if (direct) {
    // The simulator models firmware run time plus the direct protocol's
    // idle-delimited four-byte RUN command, so leave margin beyond 2 seconds.
    QThread::msleep(2500);
    if (!directConnect(sp, reconnected)) {
      fprintf(stderr, "post-flash direct reconnect failed\n");
      return 4;
    }
    printf("DIRECT SUITE PASSED\n");
  } else {
    if (!connectFourWayTarget(sp, reconnected, 0)) {
      fprintf(stderr, "post-flash 4-way reconnect failed\n");
      return 4;
    }
    printf("FOURWAY SUITE PASSED\n");
  }
  return 0;
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
            "  %s direct-suite <firmware.hex> <port>\n"
            "  %s fourway-suite <firmware.hex> <port>\n"
            "    --target / -t  4-way ESC index (default 0 = motor 1)\n"
            "defaults: port=%s\n          hex=%s\n",
            qPrintable(args[0]), qPrintable(args[0]), qPrintable(args[0]),
            qPrintable(args[0]), DEFAULT_PORT, DEFAULT_HEX);
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
  } else if (cmd == "direct-suite" || cmd == "fourway-suite") {
    if (args.size() != 4) {
      fprintf(stderr, "usage: %s %s <firmware.hex> <port>\n",
              qPrintable(args[0]), qPrintable(cmd));
      return 1;
    }
    return cmdSuite(args[2], args[3], cmd == "direct-suite");
  }
  fprintf(stderr, "unknown command '%s'\n", qPrintable(cmd));
  return 1;
}
