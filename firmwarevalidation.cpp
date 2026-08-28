#include "firmwarevalidation.h"

#include "fourwayif.h"

static uint32_t readLe32(const QByteArray &data, int offset) {
  return (uint8_t)data[offset] | ((uint8_t)data[offset + 1] << 8) |
         ((uint8_t)data[offset + 2] << 16) |
         ((uint32_t)(uint8_t)data[offset + 3] << 24);
}

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

  if (image.size() < 8)
    return fail("firmware image is too short to contain a vector table");

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

  const uint32_t stack = readLe32(image, 0);
  const uint32_t entry = readLe32(image, 4);
  // DeviceInfo does not expose the target's RAM_LIMIT_KB. Use the largest
  // limit supported by the bootloader; its tighter per-target check remains
  // the final authority.
  static const uint32_t RAM_START = 0x20000000u;
  static const uint32_t MAX_RAM_END = RAM_START + 112u * 1024u;
  if (stack < RAM_START || stack > MAX_RAM_END) {
    return fail(QString("firmware stack pointer is outside supported SRAM "
                        "(stack=0x%1)")
                    .arg(stack, 8, 16, QLatin1Char('0')));
  }

  const uint32_t entryOffset = flashOffset(entry & ~1u);
  if ((entry & 1u) == 0 || entryOffset < appOffset ||
      entryOffset >= appOffset + (uint32_t)image.size()) {
    return fail(QString("firmware entry point does not match target app start "
                        "(entry=0x%1 app=0x%2 size=%3)")
                    .arg(entry, 8, 16, QLatin1Char('0'))
                    .arg(appOffset, 8, 16, QLatin1Char('0'))
                    .arg(image.size()));
  }
  return true;
}
