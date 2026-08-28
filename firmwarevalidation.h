#ifndef FIRMWAREVALIDATION_H
#define FIRMWAREVALIDATION_H

#include <QByteArray>
#include <QString>
#include <cstdint>

class FourWayIF;

bool validateFirmwareImage(const QByteArray &image, const FourWayIF &fw,
                           bool originKnown, uint32_t origin, QString *error);

// Compare the image's linker-placed FILE_NAME block with the same block read
// from the connected ESC. This distinguishes board variants which share an
// MCU, application origin and capacity but use different pin/peripheral maps.
bool validateFirmwareTarget(const QByteArray &image, const FourWayIF &fw,
                            const QByteArray &connectedFilename,
                            QString *error);

#endif  // FIRMWAREVALIDATION_H
