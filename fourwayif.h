#ifndef FOURWAYIF_H
#define FOURWAYIF_H

#include <QMessageBox>





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


private:
    int testVar;
    char ack_req;

};

#endif // FOURWAYIF_H
