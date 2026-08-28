/*
  SerialPortConnector_CLI - headless AM32 ESC configurator used as a protocol
  testbed. It reuses the shared protocol logic in FourWayIF + hexfile so it
  cannot diverge from the GUI on protocol behaviour. Only the serial transport
  glue is CLI-local.

  Commands:
    settings <port>             read + decode the 48-byte EEPROM, save settings.dat
    flash <firmware.hex> <port> flash firmware, preserving/initialising the EEPROM
    direct-suite ...            CI exercise of the direct 1-wire protocol
    fourway-suite ...           CI exercise of 4-way discovery and protocol

  Normal commands connect through a Betaflight FC via MSP passthrough ->
  BLHeli 4-way (115200); the direct suite uses the 19200-baud 1-wire protocol.
*/
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QSerialPort>
#include <QTemporaryFile>
#include <QThread>
#include <cstdint>
#include <cstdio>

#include "BF_ROOTLOADER.h"
#include "defaults.h"
#include "firmwarevalidation.h"
#include "fourwayif.h"
#include "hexfile.h"

static const int EEPROM_PRESERVE_SIZE = 1024;
static const int PROTOCOL_CHUNK_SIZE = 256;

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
  while (timer.elapsed() < totalMs) {
    const bool ready = sp.waitForReadyRead(200);
    if (ready)
      got += sp.readAll();
    // A prior no-reply command (notably direct RUN) can become readable only
    // after this transaction starts. Locate this command's echo instead of
    // assuming it is the first buffered byte.
    const int echo = got.indexOf(cmd);
    if (echo >= 0 && got.size() >= echo + cmd.size() + replyLen)
      return got.mid(echo + cmd.size(), replyLen);
    // Some 1-wire adapters do not echo transmitted commands.  Accept only an
    // exact-sized bare reply so unrelated/stale serial bytes cannot be
    // mistaken for the current transaction.
    if (replyLen > 0 && !ready && got.size() == replyLen)
      return got;
    if (replyLen == 0 && !ready)
      return QByteArray();
  }
  return QByteArray();
}

static bool validDirectDeviceInfo(const QByteArray &info) {
  return info.size() == 9 && info[0] == '4' && info[1] == '7' &&
         info[2] == '1' && (uint8_t)info[8] == 0x30;
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
  fw.resetDeviceState();
  static const char init[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                              0, 0x0d, 'B', 'L', 'H', 'e', 'l', 'i',
                              (char)0xf4, 0x7d};
  QByteArray info = directTxn(sp, QByteArray(init, sizeof(init)), 9);
  if (!validDirectDeviceInfo(info)) {
    fprintf(stderr, "no valid device info from direct bootloader (%d bytes: %s)\n",
            info.size(), info.toHex(' ').constData());
    return false;
  }
  fw.parseDeviceInfo(info, /*direct=*/true);
  fw.direct = true;

  QByteArray devinfo = directRead(sp, 64, ADDRESS_MAGIC_DEVINFO);
  if (devinfo.size() >= 27)
    fw.parseDevinfoBlock(devinfo);

  if (!fw.addressLayoutUsable()) {
    fprintf(stderr, fw.memory_divider_required_four
                        ? "128k direct targets need a v3 bootloader\n"
                        : "unsupported flash layout and no v3 devinfo\n");
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
  fw.resetDeviceState();
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
  // Read through the maximum supported v3 struct size so future extensions
  // can be consumed without another transport change. Older bootloaders
  // reject the magic address and retain the flash-size-code defaults.
  if (fourWayTxn(sp, fw, fw.makeFourWayReadCommand(64, ADDRESS_MAGIC_DEVINFO),
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
  if (!fw.addressLayoutUsable()) {
    fprintf(stderr, fw.memory_divider_required_four
                        ? "128k 4-way targets require v3 devinfo\n"
                        : "unsupported flash layout and no v3 devinfo\n");
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

static QByteArray readRegion(QSerialPort &sp, FourWayIF &fw,
                             uint16_t firstAddress, int size,
                             bool eepromRegion = false) {
  QByteArray result;
  while (result.size() < size) {
    const int amount = qMin(PROTOCOL_CHUNK_SIZE, size - result.size());
    const uint16_t address = result.isEmpty()
                                 ? firstAddress
                                 : (eepromRegion
                                        ? fw.eepromChunkAddress(result.size())
                                        : ADDRESS_MAGIC_CONTINUE);
    QByteArray payload;
    if (g_direct) {
      payload = directRead(sp, amount, address);
    } else if (!fourWayTxn(sp, fw, fw.makeFourWayReadCommand(amount, address),
                           payload, 300, 4)) {
      return QByteArray();
    }
    if (payload.size() != amount) {
      return QByteArray();
    }
    result += payload;
  }
  return result;
}

static QByteArray readSettings(QSerialPort &sp, FourWayIF &fw) {
  return readRegion(sp, fw, fw.eepromReadAddress(), 48, true);
}

static QByteArray readFilename(QSerialPort &sp, FourWayIF &fw) {
  return readRegion(sp, fw, fw.filenameReadAddress(), 32);
}

static QByteArray readEepromImage(QSerialPort &sp, FourWayIF &fw) {
  return readRegion(sp, fw, fw.eepromReadAddress(), EEPROM_PRESERVE_SIZE, true);
}

static bool verifyFirmwareImage(QSerialPort &sp, FourWayIF &fw,
                                const QByteArray &image) {
  const int chunkSize = 256;
  for (int offset = 0; offset < image.size(); offset += chunkSize) {
    QByteArray expected = image.mid(offset, chunkSize);
    while ((expected.size() & 7) != 0)
      expected.append((char)0xff);
    QByteArray actual;
    const uint16_t address = fw.firmwareChunkAddress(offset);
    if (g_direct) {
      actual = directRead(sp, expected.size(), address);
    } else if (!fourWayTxn(sp, fw,
                           fw.makeFourWayReadCommand(expected.size(), address),
                           actual, 300, 3)) {
      return false;
    }
    if (actual != expected)
      return false;
  }
  return true;
}

static bool writeEepromImage(QSerialPort &sp, FourWayIF &fw,
                             const QByteArray &image) {
  if (image.size() != EEPROM_PRESERVE_SIZE) {
    return false;
  }
  for (int offset = 0; offset < image.size(); offset += PROTOCOL_CHUNK_SIZE) {
    const QByteArray chunk = image.mid(offset, PROTOCOL_CHUNK_SIZE);
    const uint16_t address = offset == 0 ? fw.eepromWriteAddress()
                                         : fw.eepromChunkAddress(offset);
    bool ok;
    if (g_direct) {
      ok = directWrite(sp, address, chunk);
    } else {
      QByteArray payload;
      ok = fourWayTxn(sp, fw,
                      fw.makeFourWayWriteCommand(chunk, chunk.size(), address),
                      payload, 500, 3);
    }
    if (!ok) {
      return false;
    }
  }
  return true;
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
                          const QByteArray &image, uint32_t imageOrigin,
                          uint8_t target) {
  // Reject a wrong-target, relocated, or oversized image before clearing the
  // boot bit or writing any application/configuration flash.
  QString validationError;
  if (!validateFirmwareImage(image, fw, true, imageOrigin, &validationError)) {
    fprintf(stderr, "%s\n", qPrintable(validationError));
    return 3;
  }

  // Preserve the full EEPROM state covered by the bootloader, including the
  // tune and extended parameters. A short config-only write erases the rest
  // of the physical flash page on page-aligned MCUs.
  QByteArray eep = readEepromImage(sp, fw);
  if (eep.size() != EEPROM_PRESERVE_SIZE) {
    fprintf(stderr, "failed to read EEPROM preservation region\n");
    return 4;
  }
  if ((uint8_t)eep[0] == 0x01) {
    printf("preserving existing settings and tune\n");
  } else {
    eep.replace(0, 48, QByteArray((const char *)air_starteeprom, 48));
    printf("using default settings (eeprom was blank)\n");
  }

  // Fail closed if the boot guard cannot be written.
  QByteArray e0 = eep;
  e0[0] = 0x00;
  if (!writeEepromImage(sp, fw, e0)) {
    fprintf(stderr, "pre-flash EEPROM safety write failed\n");
    return 4;
  }

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

  // Read every programmed byte back while the boot bit is still clear.  A
  // mismatch leaves the ESC safely in its bootloader instead of claiming a
  // successful flash and attempting to run a corrupt image.
  if (!verifyFirmwareImage(sp, fw, image)) {
    fprintf(stderr, "firmware readback verification failed\n");
    return 4;
  }

  // post-flash write: boot bit = 1
  QByteArray e1 = eep;
  e1[0] = 0x01;
  if (!writeEepromImage(sp, fw, e1)) {
    fprintf(stderr, "post-flash EEPROM write failed; ESC not reset\n");
    return 4;
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

  printf("FLASH PROGRAMMED AND VERIFIED; reset sent (boot execution unverified)\n");
  return 0;
}

static int cmdProtocolSelftest() {
  FourWayIF fw;
  QByteArray payload;
  const QByteArray shortFrame("\x2e\x3a\x00", 3);
  if (fw.parseFourWayResponse(shortFrame, payload))
    return 1;

  // Internally CRC-valid, but claims four payload bytes while carrying one.
  QByteArray malformed("\x2e\x3a\x00\x00\x04\xaa\x00", 7);
  const uint16_t crc = fw.makeCRC(malformed);
  malformed.append((char)(crc >> 8));
  malformed.append((char)crc);
  if (fw.parseFourWayResponse(malformed, payload))
    return 1;

  QByteArray badInfo("471\x14\x35\x06\x06\x03\xc1", 9);
  if (validDirectDeviceInfo(badInfo))
    return 1;
  QByteArray goodInfo("471\x14\x35\x06\x06\x03\x30", 9);
  if (!validDirectDeviceInfo(goodInfo))
    return 1;
  fw.resetDeviceState();
  fw.flash_layout_known = true;
  if (!fw.addressLayoutUsable())
    return 1;
  fw.memory_divider_required_four = true;
  if (fw.addressLayoutUsable())
    return 1;

  QTemporaryFile validHex;
  if (!validHex.open())
    return 1;
  validHex.write(":020000040800F2\n"
                 ":08100000001000200910000897\n"
                 ":01101000AA35\n"
                 ":00000001FF\n");
  validHex.close();
  QString hexError;
  uint32_t origin = 0;
  const QByteArray parsed = parseIntelHex(validHex.fileName(), &hexError, &origin);
  if (!hexError.isEmpty() || parsed.size() != 17 ||
      origin != 0x08001000u || (uint8_t)parsed[8] != 0xff ||
      (uint8_t)parsed[15] != 0xff || (uint8_t)parsed[16] != 0xaa) {
    return 1;
  }

  QTemporaryFile malformedHex;
  if (!malformedHex.open())
    return 1;
  malformedHex.write(":01\n");
  malformedHex.close();
  if (!parseIntelHex(malformedHex.fileName(), &hexError, &origin).isEmpty() ||
      hexError.isEmpty()) {
    return 1;
  }

  FourWayIF target;
  target.devinfo_v3.enabled = true;
  target.devinfo_v3.address_shift = 0;
  target.devinfo_v3.firmware_start = 0x1000;
  target.devinfo_v3.eeprom_start = 0x7c00;
  QString validationError;
  if (!validateFirmwareImage(parsed, target, true, 0x08001000u,
                             &validationError) ||
      validateFirmwareImage(parsed, target, true, 0x08002000u,
                            &validationError)) {
    return 1;
  }
  QByteArray badStack = parsed;
  badStack[0] = 0x00;
  badStack[1] = 0x10;
  badStack[2] = 0x00;
  badStack[3] = 0x10;
  if (validateFirmwareImage(badStack, target, true, 0x08001000u,
                            &validationError)) {
    return 1;
  }
  QByteArray oversized((int)target.applicationCapacity() + 1, (char)0xff);
  oversized.replace(0, parsed.size(), parsed);
  if (validateFirmwareImage(oversized, target, true, 0x08001000u,
                            &validationError)) {
    return 1;
  }
  printf("PROTOCOL SELFTEST PASSED\n");
  return 0;
}

static int cmdFlash(const QString &hexPath, const QString &port, uint8_t target) {
  QString err;
  uint32_t imageOrigin = 0;
  QByteArray image = parseIntelHex(hexPath, &err, &imageOrigin);
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
  return flashConnected(sp, fw, image, imageOrigin, target);
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
  uint32_t imageOrigin = 0;
  QByteArray image = parseIntelHex(hexPath, &err, &imageOrigin);
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
    FourWayIF found;
    for (int target = 0; target < escCount; target++) {
      if (!connectFourWayTarget(sp, found, (uint8_t)target))
        return 5;
      printf("ESC %d discovered (v3=%d layout_known=%d)\n", target + 1,
             (int)found.devinfo_v3.enabled, (int)found.flash_layout_known);
    }
    // Discovery leaves the last channel selected; select motor one for the
    // destructive settings/flash part of the suite.
    if (!connectFourWayTarget(sp, fw, 0))
      return 5;
  }

  QByteArray before = readEepromImage(sp, fw);
  if (before.size() != EEPROM_PRESERVE_SIZE) {
    fprintf(stderr, "failed to read EEPROM image\n");
    return 4;
  }
  QByteArray changed = before;
  changed[30] = (uint8_t)changed[30] == 5 ? 4 : 5;
  if (!writeEepromImage(sp, fw, changed)) {
    fprintf(stderr, "settings write failed\n");
    return 4;
  }
  QByteArray readback = readEepromImage(sp, fw);
  if (!settingsEqualExceptBootloaderVersion(changed, readback)) {
    fprintf(stderr, "settings write/readback mismatch\n");
    return 4;
  }
  printf("settings/tune preservation write+readback ok\n");

  int rc = flashConnected(sp, fw, image, imageOrigin, 0);
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
  } else {
    if (!connectFourWayTarget(sp, reconnected, 0)) {
      fprintf(stderr, "post-flash 4-way reconnect failed\n");
      return 4;
    }
  }
  QByteArray finalEeprom = readEepromImage(sp, reconnected);
  if (!settingsEqualExceptBootloaderVersion(changed, finalEeprom)) {
    fprintf(stderr, "EEPROM/tune changed during flash\n");
    return 4;
  }
  QByteArray finalFilename = readFilename(sp, reconnected);
  if (finalFilename.size() != 32 || !finalFilename.startsWith("AM32_")) {
    fprintf(stderr, "firmware filename read from wrong address\n");
    return 4;
  }
  printf(direct ? "DIRECT SUITE PASSED\n" : "FOURWAY SUITE PASSED\n");
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
            "  %s [--target N] settings <port>\n"
            "  %s [--target N] flash <firmware.hex> <port>\n"
            "  %s direct-suite <firmware.hex> <port>\n"
            "  %s fourway-suite <firmware.hex> <port>\n"
            "  %s protocol-selftest\n"
            "    --target / -t  4-way ESC index (default 0 = motor 1)\n",
            qPrintable(args[0]), qPrintable(args[0]), qPrintable(args[0]),
            qPrintable(args[0]), qPrintable(args[0]));
    return 1;
  }

  const QString cmd = args[1];
  if (cmd == "protocol-selftest") {
    return args.size() == 2 ? cmdProtocolSelftest() : 1;
  } else if (cmd == "settings") {
    if (args.size() != 3) {
      fprintf(stderr, "usage: %s [--target N] settings <port>\n",
              qPrintable(args[0]));
      return 1;
    }
    return cmdSettings(args[2], target);
  } else if (cmd == "flash") {
    if (args.size() != 4) {
      fprintf(stderr, "usage: %s [--target N] flash <firmware.hex> <port>\n",
              qPrintable(args[0]));
      return 1;
    }
    return cmdFlash(args[2], args[3], target);
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
