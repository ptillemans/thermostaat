#include <OneWire.h>
#include <Time.h>
#include <TinkerKit.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <EEPROM.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {  
  0x90, 0xA2, 0xDA, 0x00, 0x6D, 0x61 };
IPAddress ip(192, 168, 1, 244);


IPAddress timeServer(192, 168, 1, 1);

unsigned int localPort = 8888;      // local port to listen on

const int NTP_PACKET_SIZE= 48; // NTP time stamp is in the first 48 bytes of the message

// buffers for receiving and sending data
uint8_t packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged";       // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

int timeZoneHour = +1; // Enter Your Actual Time Zone EDT (-4)

long timeZoneOffset = (timeZoneHour * -1) * 60 * 60 ;
int NTP_Update_Interval = 60; // Number of secs before resync - should be longer just testing really



#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by unix time_t as ten ascii digits
#define TIME_HEADER  'T'   // Header tag for io. time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

#define PROGRAM_MSG_LEN 14 // progrom header + day + 4x 3digit byte.
#define PROGRAM_HEADER 'P'

#define LIST_MSG_LEN 1     // list command
#define LIST_HEADER  'L'   // 

#define STATUS_HEADER  'S'   // 


#define DEBOUNCE_MAX 120    // time to debounce the heating

OneWire  ds(O4);  // on pin 5 (a 4.7K resistor is necessary)

TKOutput heating(O5);  // creating the object 'relay' that belongs to the 'TKRlay' class 
                     // and giving the value to the desired output pin

byte addr[8];

#define TEMP_LOW   14.0;
#define TEMP_HIGH  22.0;

//
// each day looks like :
//     ________            __________
// ___/        \__________/          \____
//    T0       T1         T2         T3
// the 4 times are stored in an array for each day
// each time is encoded in a byte which is the hours
// multiplied by 10. This gives a 6 minute resolution
// 
// to disable the 2nd zone set T2=255
// to start the day 'warm' set T0=0
//
byte program[7][4]; // each doy has 2 warm zones 

#define PROGRAM_EEPROM_START 0

float temp;
float target;

int debounce = DEBOUNCE_MAX / 2;

EthernetServer server = EthernetServer(localPort);

TKButton manualHeating = TKButton(I0);
TKLed highTempLed = TKLed(O2);

#define MANUAL_HEATING_TIME 1800; 
int manualHeatingTimer = 0;

void setup(void) {
  
  Serial.begin(9600);
  
  Serial.println("Configure Ethernet using DHCP");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
   // for(;;)
   //   ;
  }
  Udp.begin(localPort);
  Serial.print("Got IP:");
  Serial.println(Ethernet.localIP());

  setSyncProvider(getNTPTime);
  Serial.println("Looking for a time");
 // while(timeStatus()== timeNotSet)   
 //    ;
  Serial.println("Got a Time");
  setSyncInterval(NTP_Update_Interval);

  getDsAddress();
 
  loadProgram();
  
  server.begin();

}

void getDsAddress() {
  byte i;

  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
}

void loop(void) {
  
  Serial.println("entering loop");
  
  processMessage();
  
  startTempMeasurement();
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  temp = readTemperature();
  target = getTargetTemperature();
  updateHeating();

  displayStatus(Serial);
}

void startTempMeasurement() {
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
}

float readTemperature(void) {
  byte i;
  byte data[12];
  
  ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  raw = raw << 3; // 9 bit resolution default
  if (data[7] == 0x10) {
    // "count remain" gives full 12 bit resolution
    raw = (raw & 0xFFF0) + 12 - data[6];
  }

  return (float)raw / 16.0;
}

void updateHeating() {
  if (temp > target) {
    debounce = debounce - 1;
    if (debounce <= 0) {
      debounce = 0;
      heating.off();
    }
  } else {
    debounce = debounce + 1;
    if (debounce >= DEBOUNCE_MAX) {
      debounce = DEBOUNCE_MAX;
      heating.on();
    }
  } 
}

void displayStatus(Stream& io) {
  digitalClockDisplay(io);
  temperatureDisplay(io, temp, target);
  heatingDisplay(io, heating.state(), debounce);
  manualHeatingDisplay(io, manualHeatingTimer);
}

void temperatureDisplay(Stream& io, float temp, float target) {
  io.print("  Temperature = ");
  io.print(temp);
  io.println(" Celsius");
  io.print("  Desired = ");
  io.print(target);
  io.println(" Celcius");
}

void heatingDisplay(Stream& io, int heating, int debounce) {
  io.print("  Heating is ");
  if (heating == LOW) {
    io.print("OFF");
  } else {
    io.print("ON");
  }
  io.print(" (");
  io.print(debounce);
  io.print("/");
  io.print(DEBOUNCE_MAX);
  io.println(")");
}

void digitalClockDisplay(Stream& io){
  // digital clock display of the time
  io.print(year()); 
  io.print("-");
  printDigits(io, month());
  io.print("-");
  printDigits(io, day());
  io.print("T");
  io.print(hour());
  io.print(":");
  printDigits(io, minute());
  io.print(":");
  printDigits(io, second());
  io.println(); 
  io.print("  Day = ");
  io.print(weekday() - 1);  // convert to 0 based
  io.println();
}

void manualHeatingDisplay(Stream& io, int timeleft) {
  io.print("  Manual Time Left = ");
  io.println(timeleft);
}


void printDigits(Stream& io, int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < 10)
    io.print('0');
  io.print(digits);
}

float getTargetTemperature() {
  byte day = weekday() - 1;
  byte compareTime = hour() * 10 + minute() / 6;
  float target = TEMP_LOW;
  boolean warm = false;

  if (manualHeating.pressed()) {
    manualHeatingTimer = MANUAL_HEATING_TIME;
    debounce = DEBOUNCE_MAX;        // override debounce delay
  }
  
  if (manualHeatingTimer > 0) {
    warm = true;
    manualHeatingTimer--;
  } else {
    for (int i = 0; i < 4; i++) {
      if (compareTime >= program[day][i]) {
        warm = !warm;
      }
    }
  }
  
  if (warm) {
    target = TEMP_HIGH;
    highTempLed.on();
  } else {
    highTempLed.off();
  }
  
  return target;
}

void processMessage() {
  char c;
  EthernetClient client = server.available();
  if (client) {
    c = client.read();
    Serial.print(c);  
 
    if (c == TIME_HEADER) {
      processSyncMessage(client);
    } else if (c == PROGRAM_HEADER) {
      processProgramMessage(client);
    } else if (c == LIST_HEADER) {
      processListMessage(client);
    } else if (c == STATUS_HEADER) {
      displayStatus(client);
    }
  }
}

void processSyncMessage(Stream& io) {
  // if time sync available from io. port, update time and return true
  time_t pctime = 0;
  char c;
  for(int i=0; i < TIME_MSG_LEN -1; i++){   
    c = io.read();                 
    if( c >= '0' && c <= '9'){   
      pctime = (10 * pctime) + (c - '0') ; // convert digits to a number    
    }
  }   
  setTime(pctime);   // Sync Arduino clock to the time received on the io. port
}

void processProgramMessage(Stream& io) {
 byte day = readDigit(io);
  if (day < 7) {
    Serial.print(" ");
    Serial.print(day);
    for (int i = 0; i < 4; i++) {
      byte b = readByte(io);
      Serial.print(" ");
      printProgramTime(io, b);
      program[day][i] = b;
    }
  }
  Serial.println();
  saveProgram();
}     

void processListMessage(Stream& io) {
   
  for(int d=0; d < 7; d++) {
    io.print("P ");
    io.print(d);
    for(int t=0; t < 4; t++) {
      printProgramTime(io, program[d][t]);
    }
    io.println();
  }
}
  
byte readDigit(Stream& io) {
  return io.read() - '0';
}

byte readByte(Stream& io) {
  int s = 0;
  for(int i = 0; i<3; i++) {
    s = s * 10 + readDigit(io);
  }
  return (byte)s;
}

void printProgramTime(Stream& io, byte t) {
  io.print(' ');
  printDigits(io, t/10);
  io.print(':');
  printDigits(io, (t % 10) * 6);
}

unsigned long getNTPTime()
{
  Serial.println("In getNTPTime");
  sendNTPpacket(timeServer);
  Serial.println("ntp request sent");
  delay(500);
  Serial.println("checking response");
  if ( Udp.parsePacket() ) {  
    Serial.println("Got Time Packet");
    Udp.read(packetBuffer,NTP_PACKET_SIZE);
 
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);  
    unsigned long secsSince1900 = highWord << 16 | lowWord;  

    const unsigned long seventyYears = 2208988800UL + timeZoneOffset;      
    unsigned long epoch = secsSince1900 - seventyYears;  
  
    return epoch;    
  }
  Serial.println("No Time Packet Found");
  return 0;
}


// send an NTP request to the time server at the given address 
unsigned long sendNTPpacket(IPAddress& address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE); 
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49; 
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp: 		   
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer,NTP_PACKET_SIZE);
  Udp.endPacket(); 
}

// store and save  program to from EEPROM

void saveProgram() {
   
  int addr = PROGRAM_EEPROM_START;
  for(int d=0; d < 7; d++) {
    for(int t=0; t < 4; t++) {
      EEPROM.write(addr++, program[d][t]);
    }
  }
  Serial.println("program saved");
}
  
  
void loadProgram() {
  int addr = PROGRAM_EEPROM_START;
  for(int d=0; d < 7; d++) {
    for(int t=0; t < 4; t++) {
      program[d][t] = EEPROM.read(addr++);
    }
  }
  Serial.println("program loaded");
} 

