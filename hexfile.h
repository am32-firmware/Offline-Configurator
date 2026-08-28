#ifndef HEXFILE_H
#define HEXFILE_H

#include <QByteArray>
#include <QString>
#include <cstdint>

/*
  Parse an Intel-HEX file into a raw flash image. The first data-record address
  is returned in origin, type-02/type-04 extended addresses are honoured, and
  gaps are filled with the erased-flash value (0xFF). Any malformed input
  returns an empty image and a diagnostic instead of a partial image.
 */
QByteArray parseIntelHex(const QString &path, QString *err,
                         uint32_t *origin = nullptr);

#endif  // HEXFILE_H
