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
#include <QFile>
#include <QElapsedTimer>
#include <cstdint>
#include <cstdio>

#include "fourwayif.h"
#include "hexfile.h"
#include "defaults.h"

static const char *DEFAULT_PORT =
    "/dev/serial/by-id/usb-Betaflight_Betaflight_STM32H743_345D344E3139-if00";
static const char *DEFAULT_HEX =
    "/home/tridge/project/UAV/AM32/AM32/obj/AM32_ARK_G431_CAN_2.20.hex";

// MSP frame builder, identical to Widget::send_mspCommand
static void sendMsp(QSerialPort &sp, uint8_t cmd, const QByteArray &payload)
{
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
static QByteArray blockingRead(QSerialPort &sp, int idleMs = 200, int totalMs = 2000)
{
    QByteArray buf;
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < totalMs) {
        if (sp.waitForReadyRead(idleMs))
            buf += sp.readAll();
        else if (!buf.isEmpty())
            break; // got a message, then went idle => complete
    }
    return buf;
}

// One 4-way transaction with retries; returns true on good ACK.
static bool fourWayTxn(QSerialPort &sp, FourWayIF &fw, const QByteArray &cmd,
                       QByteArray &payload, int writeWaitMs, int retries)
{
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

static bool openAndConnect(QSerialPort &sp, FourWayIF &fw, const QString &port,
                           uint8_t target)
{
    sp.setPortName(port);
    sp.setBaudRate(QSerialPort::Baud115200);
    sp.setDataBits(QSerialPort::Data8);
    sp.setParity(QSerialPort::NoParity);
    sp.setStopBits(QSerialPort::OneStop);
    sp.setFlowControl(QSerialPort::NoFlowControl);
    if (!sp.open(QIODevice::ReadWrite)) {
        fprintf(stderr, "failed to open %s: %s\n", qPrintable(port),
                qPrintable(sp.errorString()));
        return false;
    }
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
    if (fourWayTxn(sp, fw, fw.makeFourWayReadCommand(20, ADDRESS_MAGIC_DEVINFO),
                   block, 300, 2) &&
        fw.parseDevinfoBlock(block)) {
        printf("devinfo via magic read: version=%u firmware_start=0x%04x\n",
               fw.bootloader_version, fw.firmware_start);
    } else {
        printf("devinfo magic read unavailable; using flash-size-code defaults\n");
    }

    printf("connected: bootloader_version=%u eeprom_address=0x%04x "
           "firmware_start=0x%04x divider=%d\n",
           fw.bootloader_version, fw.eeprom_address, fw.firmware_start,
           (int)fw.memory_divider_required_four);
    return true;
}

// Read the 48-byte config (filename region + config read in one 80-byte read).
static QByteArray readSettings(QSerialPort &sp, FourWayIF &fw)
{
    QByteArray payload;
    for (int t = 0; t < 5; t++) {
        if (fourWayTxn(sp, fw, fw.makeFourWayReadCommand(80, fw.eepromReadAddress()),
                       payload, 300, 0) &&
            payload.size() >= 80) {
            payload.remove(0, 32); // strip the 32-byte file-name region
            return payload.left(48);
        }
    }
    return QByteArray();
}

static bool writeEeprom(QSerialPort &sp, FourWayIF &fw, const QByteArray &buf)
{
    QByteArray payload;
    return fourWayTxn(sp, fw,
                      fw.makeFourWayWriteCommand(buf, buf.size(), fw.eepromWriteAddress()),
                      payload, 500, 3);
}

static void printSettings(const QByteArray &c, const FourWayIF &fw)
{
    printf("--- settings ---\n");
    printf(" boot bit          : 0x%02x %s\n", (uint8_t)c[0],
           (uint8_t)c[0] == 0xFF ? "(blank/bootloader)"
                                 : (uint8_t)c[0] == 0x01 ? "(valid)" : "");
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

static int cmdSettings(const QString &port, uint8_t target)
{
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

static int cmdFlash(const QString &hexPath, const QString &port, uint8_t target)
{
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

    QSerialPort sp;
    FourWayIF fw;
    if (!openAndConnect(sp, fw, port, target))
        return 2;

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
        QByteArray payload;
        if (!fourWayTxn(sp, fw,
                        fw.makeFourWayWriteCommand(c, c.size(),
                                                   fw.firmwareChunkAddress(off)),
                        payload, 200, 8)) {
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
    fw.ack_required = true;
    sp.write(fw.makeFourWayCommand(0x35, target));
    sp.waitForBytesWritten(200);
    blockingRead(sp);

    printf("FLASH SUCCESS\n");
    return 0;
}

// Strip --target N / -t N / --target=N from args (anywhere) and return the
// requested 4-way ESC index. Defaults to 0 (motor 1).
static uint8_t extractTargetArg(QStringList &args)
{
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

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    QStringList args = app.arguments();
    const uint8_t target = extractTargetArg(args);

    if (args.size() < 2) {
        fprintf(stderr,
                "usage:\n"
                "  %s [--target N] settings [port]\n"
                "  %s [--target N] flash [firmware.hex] [port]\n"
                "    --target / -t  4-way ESC index (default 0 = motor 1)\n"
                "defaults: port=%s\n          hex=%s\n",
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
    }
    fprintf(stderr, "unknown command '%s'\n", qPrintable(cmd));
    return 1;
}
