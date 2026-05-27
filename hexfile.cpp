#include "hexfile.h"

#include <QFile>
#include <QTextStream>

QByteArray parseIntelHex(const QString &path, QString *err)
{
    QFile inputHex(path);
    uint16_t last_address = 0;
    uint16_t last_size = 0;

    QByteArray rawData;
    if (inputHex.open(QIODevice::ReadOnly)) {
        QTextStream in(&inputHex);
        while (!in.atEnd()) {
            QString line = in.readLine();
            QByteArray lineArray;
            uint16_t crc = 0;

            for (int i = 1; i < line.size(); i = i + 2) {
                QString word = line.at(i);
                word.append(line.at(i + 1));
                uint16_t num = word.toLongLong(nullptr, 16);
                crc = crc + num;
                lineArray.append(num);
            }
            if (crc % 256) {
                if (err) {
                    *err = "CRC ERROR IN HEX FILE!";
                }
                break;
            }

            // 0 size, 1 address high, 2 address low, 3 data type
            uint16_t data_type = (char)lineArray.at(3);

            if (data_type == (char)0x00) { // data
                uint16_t address = (((uint8_t)lineArray.at(1) << 8) & 0xffff) |
                                   ((uint8_t)lineArray.at(2) & 0xff);

                if ((address - last_address > last_size) && (last_size > 0)) {
                    for (int i = 0; i < address - last_address - last_size; i++) {
                        rawData.append(char(0x00));
                    }
                }
                last_address = address;
                last_size = (char)lineArray.at(0);

                lineArray.remove(0, 4);
                lineArray.chop(1);
                rawData.append(lineArray);
            }
        }
        inputHex.close();
    } else if (err) {
        *err = "could not open hex file";
    }
    return rawData;
}
