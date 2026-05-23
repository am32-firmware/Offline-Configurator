#include "fourwayif.h"

#include <QString>






FourWayIF::FourWayIF()
{

testVar = 12345;
ack_req = 1;
ack_required = true;
ack_received = false;
passthrough_started = false;
ack_type = 1;
ESC_connected = false;
bootloader_version = 0;
firmware_start = 4096; // default; overridden per-MCU or from deviceInfo (v3+)
}



QByteArray FourWayIF::makeFourWayWriteCommand(const QByteArray sendbuffer, int buffer_size, uint16_t address ){
    if (buffer_size == 256){
        buffer_size = 0;
    }
         QByteArray fourWayWriteMsgOut;
     fourWayWriteMsgOut.append((char) 0x2f);
     fourWayWriteMsgOut.append((char) 0x3b);
     fourWayWriteMsgOut.append((char) (address >> 8) & 0xff);     // address high byte
     fourWayWriteMsgOut.append((char) address & 0xff);             // address low byte

     fourWayWriteMsgOut.append((char) buffer_size);      // message length
     fourWayWriteMsgOut.append(sendbuffer);
     uint16_t writeCrc  = makeCRC(fourWayWriteMsgOut);

     uint8_t fourWayCrcHighByte = (writeCrc >> 8) & 0xff;;
     uint8_t fourWayCrcLowByte = writeCrc & 0xff;

     fourWayWriteMsgOut.append((char) fourWayCrcHighByte);
     fourWayWriteMsgOut.append((char) fourWayCrcLowByte);

     return(fourWayWriteMsgOut);

}


bool FourWayIF::checkCRC(const QByteArray data, uint16_t buffer_length){
    uint16_t crc  =0;
    if (buffer_length >2){
    for(int i = 0; i < buffer_length - 2; i++) {


    crc = crc ^ (data[i] << 8);
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x8000){
            crc = (crc << 1) ^ 0x1021;
        }
        else{
            crc = crc << 1;
        }
    }
    crc = crc & 0xffff;
}

    char fourWayCrcHighByte = (crc >> 8) & 0xff;;
    char fourWayCrcLowByte = crc & 0xff;

    qInfo("high low: %d , %d",fourWayCrcHighByte ,fourWayCrcLowByte);
    qInfo("high low: %d , %d",data[buffer_length-2] ,data[buffer_length-1]);


    if((fourWayCrcHighByte == data[buffer_length-2]) &&(fourWayCrcLowByte == data[buffer_length-1])) {
        return(true);
    }else{
        return(false);
    }
}else{
        return(false);
    }

}



QByteArray FourWayIF::makeFourWayReadCommand(int buffer_size, uint16_t address ){
    if (buffer_size == 256){
        buffer_size = 0;
    }



         QByteArray fourWayReadMsgOut;
     fourWayReadMsgOut.append((char) 0x2f);
     fourWayReadMsgOut.append((char) 0x3a);
     fourWayReadMsgOut.append((char) (address >> 8) & 0xff);     // address high byte
     fourWayReadMsgOut.append((char) address & 0xff);             // address low byte
     fourWayReadMsgOut.append((char) 0x01);
     fourWayReadMsgOut.append((char) buffer_size);      // message length
     uint16_t readCrc  = makeCRC(fourWayReadMsgOut);

     uint8_t fourWayCrcHighByte = (readCrc >> 8) & 0xff;
     uint8_t fourWayCrcLowByte = readCrc & 0xff;

     fourWayReadMsgOut.append((char) fourWayCrcHighByte);
     fourWayReadMsgOut.append((char) fourWayCrcLowByte);

     return(fourWayReadMsgOut);

}

QByteArray FourWayIF::makeFourWayReadEEPROMCommand(int buffer_size, uint16_t address ){
    if (buffer_size == 256){
        buffer_size = 0;
    }
         QByteArray fourWayReadMsgOut;
     fourWayReadMsgOut.append((char) 0x2f);
     fourWayReadMsgOut.append((char) 0x3d);
     fourWayReadMsgOut.append((char) (address >> 8) & 0xff);     // address high byte
     fourWayReadMsgOut.append((char) address & 0xff);             // address low byte
     fourWayReadMsgOut.append((char) 0x01);
     fourWayReadMsgOut.append((char) buffer_size);      // message length
     uint16_t readCrc  = makeCRC(fourWayReadMsgOut);

     uint8_t fourWayCrcHighByte = (readCrc >> 8) & 0xff;;
     uint8_t fourWayCrcLowByte = readCrc & 0xff;

     fourWayReadMsgOut.append((char) fourWayCrcHighByte);
     fourWayReadMsgOut.append((char) fourWayCrcLowByte);

     return(fourWayReadMsgOut);

}



uint16_t FourWayIF::makeCRC(const QByteArray data){
    uint16_t crc  =0;
    for(int i = 0; i < data.length(); i++) {


        crc = crc ^ (data[i] << 8);
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000){
                crc = (crc << 1) ^ 0x1021;
            }
            else{
                crc = crc << 1;
            }
        }
        crc = crc & 0xffff;
    }
    return crc;
}


QByteArray FourWayIF::makeFourWayCommand(uint8_t cmd, uint8_t device_num){
    QByteArray fourWayMsgOut;
    fourWayMsgOut.append((char) 0x2f);      // escape character PC
    fourWayMsgOut.append((char) cmd);       // 4 way command
    fourWayMsgOut.append((char) 0x00);         // address 0 for single command
    fourWayMsgOut.append((char) 0x00);         //  adress
    fourWayMsgOut.append((char) 0x01);           // payload size
    fourWayMsgOut.append((char) device_num);      // payload  ESC device number

    uint16_t msgCRC = makeCRC(fourWayMsgOut);

    char fourWayCrcHighByte = (msgCRC >> 8) & 0xff;;
    char fourWayCrcLowByte = msgCRC & 0xff;

    fourWayMsgOut.append((char) fourWayCrcHighByte);
    fourWayMsgOut.append((char) fourWayCrcLowByte);

    return(fourWayMsgOut);
}

bool FourWayIF::ACK_required(){
  return(ack_required);
}

void FourWayIF::set_Ack_req(char ackreq){
    ack_req = ackreq;
}

bool FourWayIF::ACK_received(){

}


bool FourWayIF::parseDeviceInfo(const QByteArray &data, bool direct)
{
    // 4-way responses carry a 2-byte [marker][cmd] prefix before the deviceInfo
    // payload; the direct path has none (and the caller already stripped the
    // 21-byte echo), so deviceInfo[k] lives at data[base + k].
    return parseDeviceInfoAt(data, direct ? 0 : 2);
}


bool FourWayIF::parseDevinfoBlock(const QByteArray &block)
{
    // block from ADDRESS_MAGIC_DEVINFO: magic1 (LE), magic2 (LE), deviceInfo...
    if (block.size() < 8 + 9) {
        return false;
    }
    uint32_t m1 = (uint8_t)block[0] | ((uint8_t)block[1] << 8) |
                  ((uint8_t)block[2] << 16) | ((uint32_t)(uint8_t)block[3] << 24);
    uint32_t m2 = (uint8_t)block[4] | ((uint8_t)block[5] << 8) |
                  ((uint8_t)block[6] << 16) | ((uint32_t)(uint8_t)block[7] << 24);
    if (m1 != DEVINFO_MAGIC1 || m2 != DEVINFO_MAGIC2) {
        return false;
    }
    return parseDeviceInfoAt(block, 8); // deviceInfo follows the two magic words
}


bool FourWayIF::parseDeviceInfoAt(const QByteArray &data, int base)
{
    // dump the raw deviceInfo so the testbed shows the protocol version and
    // whether the v3 firmware-start bytes survive the link
    {
        QString hex;
        for (int k = 0; k < data.size(); k++)
            hex += QString::asprintf("%02x ", (uint8_t)data[k]);
        qInfo("deviceInfo resp (base=%d size=%d): %s", base,
              (int)data.size(), qPrintable(hex));
    }

    if (data.size() <= base + 7) {
        return false;
    }
    bootloader_version = (uint8_t)data[base + 7]; // deviceInfo byte 8

    const uint8_t flashcode = (uint8_t)data[base + 4];
    bool known = true;
    if (flashcode == 0x2b) {
        qInfo("G071ESC_2KB_PAGE");
        memory_divider_required_four = true;
        eeprom_address = 0x7e00; // eeprom address of 0x1f800 126kb
        firmware_start = 4096;
    } else if (flashcode == 0x1f) {
        qInfo("F0531ESC_1KB_PAGE");
        memory_divider_required_four = false;
        eeprom_address = 0x7c00; // eeprom address of 0x7c00 31kb
        firmware_start = 4096;
    } else if (flashcode == 0x35) {
        qInfo("F3ESC_2KB_PAGE");
        memory_divider_required_four = false;
        eeprom_address = 0xF800; // eeprom address of 0xf800 62kb
        firmware_start = 4096;
    } else if (flashcode == 0x15) {
        qInfo("NXP ESC_8KB_PAGE");
        memory_divider_required_four = false;
        eeprom_address = 0xE000; // eeprom address of 64k-8k
        firmware_start = 16384;
    } else {
        qInfo("unknown flash size code 0x%02x", flashcode);
        known = false;
    }

    if (bootloader_version >= BOOTLOADER_PROTOCOL_FW_START &&
        data.size() > base + 10) {
        // v3 deviceInfo appends FIRMWARE_RELATIVE_START as a little-endian
        // uint16. Prefer it over the FLASH_SIZE_CODE guess so we flash at the
        // right offset regardless of the bootloader's flash layout (e.g. CAN
        // variants reserve more flash for the bootloader).
        firmware_start =
            (uint8_t)data[base + 9] | ((uint8_t)data[base + 10] << 8);
        qInfo("firmware_start from deviceInfo: 0x%x", firmware_start);
    }

    ESC_connected = true;
    return known;
}


bool FourWayIF::parseFourWayResponse(const QByteArray &resp, QByteArray &payloadOut)
{
    payloadOut.clear();
    if (resp.size() < 3) {
        ack_type = FW_BAD_ACK;
        return false;
    }
    if (!checkCRC(resp, resp.size())) {
        qInfo("4WAY CRC ERROR");
        ack_type = FW_CRC_ERROR;
        return false;
    }
    if (resp[resp.size() - 3] != (char)0x00) { // ACK byte (0x00 == OK)
        qInfo("ACK ERROR: cmd=0x%02x ackcode=0x%02x", (uint8_t)resp[1],
              (uint8_t)resp[resp.size() - 3]);
        ack_type = FW_BAD_ACK;
        return false;
    }

    // good ACK
    ack_required = false;
    ack_type = FW_ACK_OK;

    if (resp[1] == (char)0x3a) { // read response: payload starts at byte 5
        for (int i = 0; i < (uint8_t)resp[4]; i++) {
            payloadOut.append(resp[i + 5]);
        }
    }
    if (resp[1] == (char)0x37) { // deviceInfo response
        parseDeviceInfo(resp, false);
    }
    return true;
}


uint16_t FourWayIF::eepromReadAddress() const
{
    if (bootloader_version >= BOOTLOADER_PROTOCOL_MAGIC_ADDR) {
        return ADDRESS_MAGIC_FILE_NAME;
    }
    return eeprom_address - 32;
}


uint16_t FourWayIF::eepromWriteAddress() const
{
    if (bootloader_version >= BOOTLOADER_PROTOCOL_MAGIC_ADDR) {
        return ADDRESS_MAGIC_EEPROM;
    }
    return eeprom_address;
}


uint16_t FourWayIF::firmwareChunkAddress(uint32_t offset) const
{
    uint32_t addr = firmware_start + offset;
    if (memory_divider_required_four) {
        addr >>= 2;
    }
    return (uint16_t)addr;
}



