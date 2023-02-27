#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <SoftwareSerial.h>

#define ID 0x01
#define bufferSize 7

// For RS232
#define RXD2 16
#define TXD2 17

// RS485 setup with ESp32
#define RE 32  // Connect RE terminal with 32 of ESP
#define DE 33    // Connect DE terminal with 33 of ESP       
     
//const byte ModReadBuffer[] = {0x01, 0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
byte ModReadBuffer[bufferSize] = {ID, 0x03, 02, 0x00, 0x41}; // packet to be given in return Id FC numOfBytes dataMSB dataLSB CRCMSB CRCLSB
byte BufferValue[bufferSize];
SoftwareSerial mod(26, 27); // RX=26 , TX =27

static int weightData(char*);
static int weight = 0;

void setup() {
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.println("serial2test");
  Serial.println("Serial Txd is on pin: " + String(TX));
  Serial.println("Serial Rxd is on pin: " + String(RX));

//    Serial.begin(115200);
  mod.begin(9600);// modbus configuration
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
}

void loop() {
  fun_RS232();
//  delay(100);
  fun_RS485();
}

// To measure the weight from RS232
int weightData(char *weightString){
  char *p = weightString;
  int arr[6];
  int val=0;
  while (*p) {
      if (isdigit(*p)) {
          val = strtol(p, &p, 10);
//          printf("%ld\n", val);
      } else {
          p++;
      }
  }
  return val;
}

char c;
String readString;
void fun_RS232(){
   while (Serial2.available()) {
    c = Serial2.read();
    readString += c;
  }
  if (readString.length() > 0) {
//    Serial.print(readString);
    char weightStr[8];
    readString.toCharArray(weightStr, 8);
    weight = weightData(weightStr);
//    Serial.println(weight);
   
    readString = "";
    }
}

unsigned int crc_16 = 0;
unsigned int MatchBuffer[2] = {0};
void fun_RS485(){
  byte val1;
  crc_16 = crc_calc(ModReadBuffer, 6);
  ModReadBuffer[6] = crc_16 & 0x00FF;
  ModReadBuffer[7] =(( crc_16 >> 8) & 0x00FF);
  val1 = ModbusData();
  delay(1000);
}

byte ModbusData(){
    byte i;
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(i=0;i<=8;i++){
    //Serial.print(mod.read(),HEX);
    BufferValue[i] = mod.read();
    }
    if(BufferValue[0]!=255 && BufferValue[1]!=255 && BufferValue[2]!=255 && BufferValue[3]!=255 && BufferValue[4]!=255 && 
    BufferValue[5]!=255 && BufferValue[6]!=255 && BufferValue[7]!=255){
    Serial.println("Serial Data received On:"); 
    Serial.print("Modbus Buffer[0]="); 
    Serial.println(BufferValue[0],DEC);
    Serial.print("Modbus Buffer[1]="); 
    Serial.println(BufferValue[1],DEC);
    Serial.print("Modbus Buffer[2]="); 
    Serial.println(BufferValue[2],DEC);
    Serial.print("Modbus Buffer[3]="); 
    Serial.println(BufferValue[3],DEC);
    Serial.print("Modbus Buffer[4]="); 
    Serial.println(BufferValue[4],DEC);
    Serial.print("Modbus Buffer[5]="); 
    Serial.println(BufferValue[5],DEC);
    Serial.print("Modbus Buffer[6]="); 
    Serial.println(BufferValue[6],DEC);
    Serial.print("Modbus Buffer[7]="); 
    Serial.println(BufferValue[7],DEC);
    Serial.print("weight=");
    Serial.println(weight);
    // For checking the CRC if the data is lost or not
    crc_16 = crc_calc(BufferValue, 6);
    MatchBuffer[0] = crc_16 & 0x00FF;
    MatchBuffer[1] =(( crc_16 >> 8) & 0x00FF);
    Serial.print("MatchBuffer[0]="); 
    Serial.println(MatchBuffer[0],DEC);
    Serial.print("MatchBuffer[1]="); 
    Serial.println(MatchBuffer[1],DEC);
    
    // If ID and CRC of the slave matches
    if(BufferValue[0] == ID && (MatchBuffer[0] = BufferValue[6]) && (MatchBuffer[1] = BufferValue[7])){
        ModReadBuffer[3] = weight & 0x00FF;
        ModReadBuffer[4] = (weight>>8) & 0x00FF;
        crc_16 = crc_calc(ModReadBuffer, 5); // Id FC numOfBytes dataMSB dataLSB 
        MatchBuffer[0] = crc_16 & 0x00FF;
        MatchBuffer[1] =(( crc_16 >> 8) & 0x00FF);
        ModReadBuffer[5] = MatchBuffer[0];
        ModReadBuffer[6] = MatchBuffer[1];
        digitalWrite(DE,HIGH);
        digitalWrite(RE,HIGH);
        mod.write(ModReadBuffer,sizeof(ModReadBuffer));
        delay(10);
        digitalWrite(DE,LOW);
        digitalWrite(RE,LOW);
        
      }
    
    Serial.println("");
//    Serial.flush();

     }
   return BufferValue[bufferSize];
  }
  
// 16bit CRC generator Function
uint16_t crc_calc(uint8_t *pcode , size_t len) {
  uint16_t crc = 0xFFFF; //Inizializzazione del registro CRC con tutti '1'
  int z;
  int y;
  for (z = 0; z < len; z++) {
    crc = crc ^ (uint16_t)pcode[z];
    for (y = 0; y < 8; y++) {
      if ((crc & 0x0001) != 0) crc = (crc >> 1) ^ 0xA001;
      else crc = (crc >> 1);
    }
  }
  return crc;
}


  
