/*
Author: hxhx
Email: haixun99@gmail.com
Version: v1.0

This program works with teensy 3.2

Current Progress:
1. implemented hardware serial communication (receive)
2. implemented checksum fletcher 16
3. Sending 

TODO:
1. Buttons/Emergency 
2. State machines (control cycles)
4. DAC to control throttle
*/

#define HWSERIAL Serial1
#define BUFFER_SIZE 256

typedef enum _serial_state {
  SER_HEADER1, SER_HEADER2, SER_DATA
} serial_state_t;

//initialize receiver state
serial_state_t serial_receiver_state = SER_HEADER1;

//IntervalTimer object
const int ledPin = LED_BUILTIN;
int ledState = LOW;

//data processing
String inData;
int expected_packet_size = 12;

int packet_count = 0;
volatile byte temp[BUFFER_SIZE];
volatile byte received_data[BUFFER_SIZE];

//sending Data
int send_data_size = 16;   // 4 x size of int
int send_packet_size = 20; // (header1, header2, 4xsizeofint, fletcher1,fletcher2)

//commands
int last_speedcmd = 0;
int last_gearshift = 0;
int last_pc_status = 0;

//penalties
int serial_penalty = 0;

void setup() {
  pinMode(ledPin,OUTPUT);
  
  Serial.begin(9600);
  HWSERIAL.begin(1500000);    //hardware serial on Serial1
}

void loop() {
    

    int speedCmd, gearShift, pc_status;
    
    // Read data
    bool success = false;
    while (HWSERIAL.available()) {
       success =  serialreceive();
    }
    if (success) {
      // unpack data
      speedCmd = received_data[0] | ( (int)received_data[1] << 8 ) | ( (int)received_data[2] << 16 ) | ( (int)received_data[3] << 24 );
      gearShift = received_data[4] | ( (int)received_data[5] << 8 ) | ( (int)received_data[6] << 16 ) | ( (int)received_data[7] << 24 );
      pc_status = received_data[8] | ( (int)received_data[9] << 8 ) | ( (int)received_data[10] << 16 ) | ( (int)received_data[11] << 24 );

      //debug receiving
      Serial.print("Speed: ");
      Serial.println(speedCmd);
      Serial.print("Gear: ");
      Serial.println(gearShift);
      Serial.print("Status: ");
      Serial.println(pc_status);

      serial_penalty = 0;          //reset serial penalty
      last_speedcmd = speedCmd;
      last_gearshift = gearShift;
      last_pc_status = pc_status;

      if(ledState == LOW){
         ledState = HIGH;
         digitalWrite(ledPin,ledState);
      }
      
    }else{

      serial_penalty++;
      speedCmd = last_speedcmd;
      gearShift = last_gearshift;
      pc_status = last_pc_status;
      
      if(ledState == HIGH){
         ledState = LOW;
         digitalWrite(ledPin,ledState);
      }
    }

    
    
    
    if(success){
      byte packet[send_packet_size];
      
      //Send data
      int button_state = 757; 
      int controller_state = 836;
      int serial_penalty = 1244;
      int version_no = 105;
  
      int packet_size = 0;
      serialAddInt(packet, &packet_size, &button_state);
      serialAddInt(packet, &packet_size, &controller_state); 
      serialAddInt(packet, &packet_size, &serial_penalty);
      serialAddInt(packet, &packet_size, &version_no);
      packData(packet,&packet_size);
      
      //Serial.println("Byte Formatting: ");
      for(int i = 0; i < send_packet_size; i++)
           HWSERIAL.write(packet[i]);
           
    }

}

void serialAddInt(uint8_t *packet, int* packet_size, int *data){
  int new_packet_size = *packet_size + sizeof(int);
  memcpy(packet+ (*packet_size), data, sizeof(int));
  *packet_size = new_packet_size;
}

void packData(uint8_t *packet, int* packet_size){
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



// Serial Receive function  (Within Serial Receive)
bool serialreceive() {

  char packet_char;
  byte packet_data;
  bool aval = false;
  
  switch (serial_receiver_state) {
    case SER_HEADER1:

      packet_char = HWSERIAL.read();
      if (packet_char == '!') {
        serial_receiver_state = SER_HEADER2;
        packet_count = 0;
      }
      break;
    case SER_HEADER2:

      packet_char = HWSERIAL.read();
      if (packet_char == '#') {
        serial_receiver_state = SER_DATA;
        packet_count  = 0;
      }
      else {
        serial_receiver_state = SER_HEADER1;
        packet_count  = 0;
      }
      break;
    case SER_DATA:
      packet_data = HWSERIAL.read();
      
      temp[packet_count] = packet_data;
      packet_count++;

      if(packet_count == expected_packet_size + 2){
          //2 bytes before LF are Fletcher's checksum
          int packet_size = expected_packet_size;
          uint8_t data_packet[packet_size];
          for (int j = 0; j < packet_size; j++)
            data_packet[j] = temp[j];
            
          uint16_t csum = fletcher16(data_packet, packet_size);
          uint16_t ver_csum = temp[packet_size] << 8 | temp[packet_size+1];
          if(csum == ver_csum){
            aval = true;
            for(int j=0; j<packet_size; j++)
              received_data[j] = data_packet[j];
          }
          serial_receiver_state = SER_HEADER1;
          packet_count = 0;
        }
      break;
  }

  return aval;
}

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

