#include "BF_ROOTLOADER.h"

#include <QByteArray>

typedef union __attribute__ ((packed)) {
    uint8_t bytes[2];
    uint16_t word;
} uint8_16_u;



BF_ROOTLOADER::BF_ROOTLOADER()
{
    ack_req = 1;
    ack_required = true;
    ack_received = false;
    passthrough_started = false;
    ack_type = 1;
    ESC_connected = false;
    calculated_crc_low_byte = 0;
    calculated_crc_high_byte = 0;
}


bool BF_ROOTLOADER::checkCRC(const QByteArray pBuff, uint16_t length){


        uint8_t received_crc_low_byte2 = pBuff[length-2];          // one higher than len in buffer
        uint8_t received_crc_high_byte2 = pBuff[length-1];
        makeCRC(pBuff,length-2);


        if((calculated_crc_low_byte==received_crc_low_byte2)   && (calculated_crc_high_byte==received_crc_high_byte2)){

          return true;
        }else{
          return false;
        }
    }


QByteArray BF_ROOTLOADER::setAddress(uint16_t address){
    QByteArray thisAddress;
    thisAddress.append((char)0xff);       // set address
    thisAddress.append((char)0x00);
    thisAddress.append((char)((address >> 8) & 0xff));
    thisAddress.append((char)(address & 0xff));
    makeCRC(thisAddress, 4);
    thisAddress.append(calculated_crc_low_byte);
    thisAddress.append(calculated_crc_high_byte);

    return thisAddress;
}

QByteArray BF_ROOTLOADER::setBufferSize(uint16_t size){
if(size == 256){
    size = 0;
}

    QByteArray output_to_esc_buffer;
    output_to_esc_buffer.append((char)0xfe);
    output_to_esc_buffer.append((char) 0x00);
    output_to_esc_buffer.append((char)0x00);
    output_to_esc_buffer.append((char)(uint8_t)size);
     makeCRC(output_to_esc_buffer, 4);
    output_to_esc_buffer.append((char)calculated_crc_low_byte);
    output_to_esc_buffer.append((char)calculated_crc_high_byte);
   return output_to_esc_buffer;
}

QByteArray BF_ROOTLOADER::writeFlash(){
    QByteArray output_to_esc_buffer;
    output_to_esc_buffer.append((char) 0x01);
    output_to_esc_buffer.append((char)0x01);
    makeCRC(output_to_esc_buffer, 2);
    output_to_esc_buffer.append((char)calculated_crc_low_byte);
    output_to_esc_buffer.append((char)calculated_crc_high_byte);
  return output_to_esc_buffer;
}

QByteArray BF_ROOTLOADER::readFlash(uint8_t size){
    QByteArray output_to_esc_buffer;
    output_to_esc_buffer.append((char)0x03);
    output_to_esc_buffer.append((char)size);

     makeCRC(output_to_esc_buffer, 2);
     output_to_esc_buffer.append((char)calculated_crc_low_byte);
     output_to_esc_buffer.append((char)calculated_crc_high_byte);

return output_to_esc_buffer;
}

QByteArray BF_ROOTLOADER::sendBuffer(QByteArray inbuffer){
     QByteArray outBuffer;
        outBuffer= inbuffer;


     makeCRC(outBuffer, outBuffer.size());
     outBuffer.append( calculated_crc_low_byte);
     outBuffer.append( calculated_crc_high_byte);

return outBuffer;
}


void BF_ROOTLOADER::makeCRC(const QByteArray pBuff, uint16_t length){
      static uint8_16_u CRC_16;
        CRC_16.word=0;

        for(int i = 0; i < length; i++) {


             uint8_t xb = pBuff[i];
             for (uint8_t j = 0; j < 8; j++)
             {
                 if (((xb & 0x01) ^ (CRC_16.word & 0x0001)) !=0 ) {
                     CRC_16.word = CRC_16.word >> 1;
                     CRC_16.word = CRC_16.word ^ 0xA001;
                 } else {
                     CRC_16.word = CRC_16.word >> 1;
                 }
                 xb = xb >> 1;
             }
         }
        calculated_crc_low_byte = CRC_16.bytes[0];
        calculated_crc_high_byte = CRC_16.bytes[1];

    }

