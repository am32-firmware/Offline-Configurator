#ifndef FIRMWAREVALIDATION_H
#define FIRMWAREVALIDATION_H

#include <QByteArray>
#include <QString>
#include <cstdint>

class FourWayIF;

bool validateFirmwareImage(const QByteArray &image, const FourWayIF &fw,
                           bool originKnown, uint32_t origin, QString *error);

#endif  // FIRMWAREVALIDATION_H
