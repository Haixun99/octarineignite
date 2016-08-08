#ifndef simple_serial_framing
#define simple_serial_framing

#include "Arduino.h"
#include "stdio.h"

#define BUFFER_SIZE 256

typedef enum _serial_state{
  SER_HEADER1, SER_HEADER2, SER_DATA
}serial_state_t;

serial_state_t serial_receiver_state = SER_HEADER1;
volatile uint8_t received_data[BUFFER_SIZE];
volatile uint8_t data_buffer[BUFFER_SIZE];
int packet_received = 0;
int expected_packet_size = 10;

//http://en.wikipedia.org/wiki/Fletcher%27s_checksum
uint16_t fletcher16( uint8_t *data, size_t bytes){
  uint16_t sum1 = 0xff, sum2 = 0xff;

  while (bytes) {
    size_t tlen = bytes > 20 ? 20 : bytes;
    bytes -= tlen;
    do {
        sum2 += sum1 += *data++;
    } while (--tlen);
    sum1 = (sum1 & 0xff) + (sum1 >> 8);
    sum2 = (sum2 & 0xff) + (sum2 >> 8);
  }
  /* Second reduction step to reduce sums to 8 bits */
  sum1 = (sum1 & 0xff) + (sum1 >> 8);
  sum2 = (sum2 & 0xff) + (sum2 >> 8);
  
  uint16_t sumsum = sum2 << 8 | sum1;
  return sumsum;
}

int serialReceive(uint8_t *packet, int packet_size){
  int available_packet = 0;
  for(int i=0; i<packet_size; i++){
//     std::cout<<i<<"/"<<packet_size<<" "<<(int)packet[i]<<" "<<serial_receiver_state<<std::endl;
    switch(serial_receiver_state){
      case SER_HEADER1:
        if(packet[i]=='!'){
          serial_receiver_state = SER_HEADER2;
    packet_received = 0;
        }
        break;
      case SER_HEADER2:
        if(packet[i]=='#'){
          serial_receiver_state = SER_DATA;
    packet_received = 0;
        }
        else {
          serial_receiver_state = SER_HEADER1;
    packet_received = 0;
  }
        break;
      case SER_DATA:
  //had lot's of problem doing the following line
  //the data got changed without assigning it!
  //Apparently the assembly rearranging data stored and it got overwritten
  //Used volatile qualifier solve it (Sort of reserving the memory buffer)
  
  data_buffer[packet_received] = packet[i];
  packet_received++;
        //ensure there is enough packet to process. There might be a chance
        //that '\n' occur on the first byte, which is what exactly happened
        //to 0.09
        if(packet_received == expected_packet_size + 2){
          //2 bytes before LF are Fletcher's checksum
          int packet_size = expected_packet_size;
          uint8_t data_packet[packet_size];
          for (int j = 0; j < packet_size; j++)
            data_packet[j] = data_buffer[j];
          uint16_t csum = fletcher16(data_packet, packet_size);
          uint16_t ver_csum = data_buffer[packet_size] << 8 | data_buffer[packet_size+1];
          if(csum == ver_csum){
            available_packet = 1;
            for(int j=0; j<packet_size; j++)
              received_data[j] = data_packet[j];
          }
          serial_receiver_state = SER_HEADER1;
          packet_received = 0;
        }
  break;
    }
  }
  return available_packet;
}


void serialAddFloat(uint8_t *packet, int *packet_size, float data){
  int new_packet_size = *packet_size + sizeof(float);
  memcpy(packet+ (*packet_size), &data, sizeof(float));
  *packet_size = new_packet_size;
}

void serialAddInt(uint8_t *packet, int *packet_size, int data){
  int new_packet_size = *packet_size + sizeof(int);
  memcpy(packet+ (*packet_size), &data, sizeof(int));
  *packet_size = new_packet_size;
}

void serialAddUInt32(uint8_t *packet, int *packet_size, uint32_t data){
  int new_packet_size = *packet_size + sizeof(uint32_t);
  memcpy(packet+ (*packet_size), &data, sizeof(uint32_t));
  *packet_size = new_packet_size;
}

void packData(uint8_t *packet, int *packet_size){
  int new_packet_size = *packet_size + 4;
  uint16_t fletcher_sum = fletcher16(packet, *packet_size);
  uint8_t packet_temp[*packet_size];
  memcpy(packet_temp, packet, *packet_size); 
  packet[0] = '!';
  packet[1] = '#';
  memcpy(packet+2, packet_temp, *packet_size);
  packet[*packet_size + 2] = (fletcher_sum >> 8)&0xff;
  packet[*packet_size + 3] = fletcher_sum&0xff;
  //packet[*packet_size + 4] = '\n';
  *packet_size = new_packet_size;
}

void unpackDataFloat(uint8_t *packet, int *packet_size, float *data){
  int new_packet_size = *packet_size - sizeof(float);
  float data_t;
  memcpy(&data_t, packet, sizeof(float));
  *data = data_t;
  if(new_packet_size > 0) {
    memmove(packet, packet+sizeof(float), new_packet_size);
  }
  *packet_size = new_packet_size;
}

void unpackDataInt(uint8_t *packet, int *packet_size, int *data){
  int new_packet_size = *packet_size - sizeof(int);
  int data_t;
  memcpy(&data_t, packet, sizeof(int));
  *data = data_t;
  if(new_packet_size > 0) {
    memmove(packet, packet+sizeof(int), new_packet_size);
  }
  *packet_size = new_packet_size;
}

void unpackDataUInt32(uint8_t *packet, int *packet_size, uint32_t *data){
  int new_packet_size = *packet_size - sizeof(uint32_t);
  uint32_t data_t;
  memcpy(&data_t, packet, sizeof(uint32_t));
  *data = data_t;
  if(new_packet_size > 0) {
    memmove(packet, packet+sizeof(uint32_t), new_packet_size);
  }
  *packet_size = new_packet_size;
}

#endif
