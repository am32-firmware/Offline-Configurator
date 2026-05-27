#ifndef HEXFILE_H
#define HEXFILE_H

#include <QByteArray>
#include <QString>

/*
  Parse an Intel-HEX file into a raw flash image (zero-filling gaps between
  records). On a checksum error, *err (if non-null) is set and whatever was
  parsed so far is returned (matching the GUI's original behaviour). Shared by
  the GUI (Widget::convertFromHex) and the CLI so hex parsing cannot diverge.
 */
QByteArray parseIntelHex(const QString &path, QString *err);

#endif // HEXFILE_H
