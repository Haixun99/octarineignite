#include <SimpleSerialProtocol.h>

SimpleSerialProtocol p;

#define HWSERIAL Serial1

void setup() {
  
  analogWriteResolution(12);
}

void loop() {
  analogWrite(A14, 0);

  delay(2000);
  analogWrite(A14,2048);
  delay(2000);
}

