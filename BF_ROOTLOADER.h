#ifndef BF_ROOTLOADER_H
#define BF_ROOTLOADER_H

#endif // BF_ROOTLOADER_H
#include <QByteArray>

class BF_ROOTLOADER{
public:
    BF_ROOTLOADER();
    QByteArray setAddress(uint16_t address);
    QByteArray setBufferSize(uint16_t size);
    QByteArray writeFlash();
    QByteArray sendBuffer(QByteArray inbuffer);
    QByteArray readFlash(uint8_t size);
    void makeCRC(const QByteArray pBuff, uint16_t length);
    uint16_t eeprom_address;

    bool checkCRC(const QByteArray pBuff , uint16_t length);
    bool ACK_required();
    bool ACK_received();
    void set_Ack_req(char ackreq);
    uint8_t connected_motor;
    bool memory_divider_required_four;
    bool ack_required;
    bool ack_received;
    bool passthrough_started;
    bool ESC_connected;

    uint8_t ack_type;


private:
    int testVar;
    char ack_req;
    uint8_t calculated_crc_low_byte;
    uint8_t calculated_crc_high_byte;
};
