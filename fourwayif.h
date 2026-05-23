#ifndef FOURWAYIF_H
#define FOURWAYIF_H

#include <QByteArray>

/*
  AM32 bootloaders report a protocol version as byte 8 of the deviceInfo.
  v2: magic flash addresses (ADDRESS_MAGIC_*) supported, so the configurator
      can address the EEPROM without knowing the per-MCU flash layout.
  v3: deviceInfo extended with a little-endian uint16 FIRMWARE_RELATIVE_START,
      so firmware can be flashed without knowing the per-variant flash layout.
  These constants are shared by the GUI and the CLI.
 */
#define BOOTLOADER_PROTOCOL_MAGIC_ADDR 2 // magic flash addresses supported
#define BOOTLOADER_PROTOCOL_FW_START 3   // deviceInfo carries firmware start
#define ADDRESS_MAGIC_EEPROM 0x20    // maps to the EEPROM config region
#define ADDRESS_MAGIC_FILE_NAME 0x21 // maps to the file-name region (EEPROM-32)
#define ADDRESS_MAGIC_DEVINFO 0x23   // maps to the devinfo struct (magic1/2 + deviceInfo)

// magic values at the start of the devinfo struct, used to confirm a 4-way
// READ at ADDRESS_MAGIC_DEVINFO returned a real devinfo block
#define DEVINFO_MAGIC1 0x5925e3da
#define DEVINFO_MAGIC2 0x4eb863d9


class FourWayIF
{
public:
    FourWayIF();
    QByteArray makeFourWayWriteCommand(const QByteArray sendbuffer, int buffer_size, uint16_t address);
    QByteArray makeFourWayReadCommand( int buffer_size, uint16_t address );
    QByteArray makeFourWayReadEEPROMCommand( int buffer_size, uint16_t address );
    QByteArray makeFourWayCommand(uint8_t cmd, uint8_t device_num);
    uint16_t makeCRC(const QByteArray data);
    uint16_t eeprom_address;
    uint16_t firmware_start;
    uint8_t bootloader_version; // protocol version from deviceInfo byte 8
    bool checkCRC(const QByteArray data , uint16_t buffer_length);
    bool ACK_required();
    bool ACK_received();
    void set_Ack_req(char ackreq);
    uint8_t connected_motor;
    bool memory_divider_required_four;
    bool ack_required;
    bool ack_received;
    bool passthrough_started;
    bool ESC_connected;
    bool direct;

    uint8_t ack_type;

    // ack_type values; integers match the GUI's Widget::messages enum so the
    // existing widget.cpp comparisons keep working after the shared refactor.
    enum { FW_ACK_OK = 0, FW_BAD_ACK = 1, FW_CRC_ERROR = 2 };

    /*
      Shared protocol-decision logic, used by both the GUI and the CLI so they
      cannot diverge on protocol behaviour.
    */
    // Parse a deviceInfo response, setting bootloader_version, eeprom_address,
    // firmware_start, memory_divider_required_four and ESC_connected. The 4-way
    // and direct framings put deviceInfo at different offsets (direct has no
    // 2-byte 4-way prefix); for direct the caller must have already stripped the
    // 21-byte echo. Returns true if the flash-size code was recognised.
    bool parseDeviceInfo(const QByteArray &data, bool direct);

    // Parse a devinfo block read from ADDRESS_MAGIC_DEVINFO: [magic1][magic2]
    // [deviceInfo...]. Validates the magic values, then parses the deviceInfo
    // (version + firmware_start), which is how the version/firmware-start are
    // obtained over a 4-way passthrough. Returns true if the block was valid.
    bool parseDevinfoBlock(const QByteArray &block);

    // Validate a 4-way response (CRC + ACK byte at size-3). On a read response
    // (0x3a) fills payloadOut with data[4] bytes from data[5..]; on a deviceInfo
    // response (0x37) calls parseDeviceInfo. Sets ack_required/ack_type. Returns
    // true on a good ACK.
    bool parseFourWayResponse(const QByteArray &resp, QByteArray &payloadOut);

    // EEPROM region addresses, using the bootloader magic addresses when the
    // protocol version supports them.
    uint16_t eepromReadAddress() const;  // file-name region (EEPROM-32)
    uint16_t eepromWriteAddress() const; // EEPROM config region

    // Absolute client flash address for a firmware chunk at the given byte
    // offset, honouring the >>2 address shift on divider MCUs.
    uint16_t firmwareChunkAddress(uint32_t offset) const;

private:
    // parse deviceInfo bytes where deviceInfo[0] is at data[base]
    bool parseDeviceInfoAt(const QByteArray &data, int base);
    int testVar;
    char ack_req;

};

#endif // FOURWAYIF_H
