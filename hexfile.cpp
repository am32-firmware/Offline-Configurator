#include "hexfile.h"

#include <QFile>
#include <QTextStream>
#include <limits>

QByteArray parseIntelHex(const QString &path, QString *err, uint32_t *origin) {
  if (err)
    err->clear();
  if (origin)
    *origin = 0;

  QFile inputHex(path);
  if (!inputHex.open(QIODevice::ReadOnly)) {
    if (err)
      *err = "could not open hex file";
    return QByteArray();
  }

  QByteArray rawData;
  uint32_t upperAddress = 0;
  uint32_t nextAddress = 0;
  bool haveData = false;
  bool haveEof = false;
  int lineNumber = 0;
  QTextStream in(&inputHex);

  auto fail = [&](const QString &message) {
    if (err)
      *err = QString("HEX line %1: %2").arg(lineNumber).arg(message);
    return QByteArray();
  };

  while (!in.atEnd()) {
    const QString line = in.readLine().trimmed();
    lineNumber++;
    if (line.isEmpty())
      continue;
    if (haveEof)
      return fail("data found after EOF record");
    if (!line.startsWith(':') || ((line.size() - 1) & 1) != 0 ||
        line.size() < 11) {
      return fail("malformed record");
    }

    QByteArray record;
    record.reserve((line.size() - 1) / 2);
    for (int i = 1; i < line.size(); i += 2) {
      bool ok = false;
      const uint value = line.mid(i, 2).toUInt(&ok, 16);
      if (!ok || value > 0xff)
        return fail("non-hexadecimal byte");
      record.append((char)value);
    }

    const int byteCount = (uint8_t)record[0];
    if (record.size() != byteCount + 5)
      return fail("byte count does not match record length");
    uint8_t checksum = 0;
    for (char byte : record)
      checksum = (uint8_t)(checksum + (uint8_t)byte);
    if (checksum != 0)
      return fail("checksum mismatch");

    const uint16_t address = ((uint8_t)record[1] << 8) | (uint8_t)record[2];
    const uint8_t type = (uint8_t)record[3];
    const QByteArray data = record.mid(4, byteCount);

    if (type == 0x00) {
      const uint64_t absolute64 = (uint64_t)upperAddress + address;
      if (absolute64 > std::numeric_limits<uint32_t>::max() ||
          absolute64 + (uint32_t)byteCount >
              (uint64_t)std::numeric_limits<uint32_t>::max()) {
        return fail("address overflow");
      }
      const uint32_t absolute = (uint32_t)absolute64;
      if (!haveData) {
        haveData = true;
        nextAddress = absolute;
        if (origin)
          *origin = absolute;
      }
      if (absolute < nextAddress)
        return fail("overlapping or out-of-order data record");
      const uint64_t gap = (uint64_t)absolute - nextAddress;
      static const uint64_t MAX_IMAGE_SIZE = 1024u * 1024u;
      if ((uint64_t)rawData.size() + gap + (uint32_t)byteCount >
          MAX_IMAGE_SIZE) {
        return fail("image span is unreasonably large");
      }
      if (gap)
        rawData.append(QByteArray((int)gap, (char)0xff));
      rawData.append(data);
      nextAddress = absolute + (uint32_t)byteCount;
    } else if (type == 0x01) {
      if (byteCount != 0 || address != 0)
        return fail("invalid EOF record");
      haveEof = true;
    } else if (type == 0x02) {
      if (byteCount != 2 || address != 0)
        return fail("invalid extended segment address record");
      upperAddress = (((uint8_t)data[0] << 8) | (uint8_t)data[1]) << 4;
    } else if (type == 0x04) {
      if (byteCount != 2 || address != 0)
        return fail("invalid extended linear address record");
      upperAddress = (uint32_t)(((uint8_t)data[0] << 8) |
                                (uint8_t)data[1])
                     << 16;
    } else if (type == 0x03 || type == 0x05) {
      if (byteCount != 4)
        return fail("invalid start-address record");
    } else {
      return fail(QString("unsupported record type 0x%1")
                      .arg(type, 2, 16, QLatin1Char('0')));
    }
  }

  if (!haveEof)
    return fail("missing EOF record");
  if (!haveData)
    return fail("file contains no data records");
  return rawData;
}
