#include "firmwarevalidation.h"

#include "fourwayif.h"

static uint32_t flashOffset(uint32_t address) {
  return address >= 0x08000000u ? address - 0x08000000u : address;
}

bool validateFirmwareImage(const QByteArray &image, const FourWayIF &fw,
                           bool originKnown, uint32_t origin, QString *error) {
  if (error)
    error->clear();
  auto fail = [&](const QString &message) {
    if (error)
      *error = message;
    return false;
  };

  if (image.isEmpty())
    return fail("firmware image is empty");

  const uint32_t appOffset = fw.applicationOffset();
  const uint32_t capacity = fw.applicationCapacity();
  if (capacity == 0 || !fw.firmwareImageFits((uint32_t)image.size())) {
    return fail(QString("firmware image is too large for the application region "
                        "(size=%1 capacity=%2)")
                    .arg(image.size())
                    .arg(capacity));
  }

  if (originKnown && flashOffset(origin) != appOffset) {
    return fail(QString("firmware HEX origin does not match target app start "
                        "(origin=0x%1 app=0x%2)")
                    .arg(origin, 8, 16, QLatin1Char('0'))
                    .arg(appOffset, 8, 16, QLatin1Char('0')));
  }

  // Startup bytes are architecture-specific; do not infer a CPU or vector format.
  return true;
}
