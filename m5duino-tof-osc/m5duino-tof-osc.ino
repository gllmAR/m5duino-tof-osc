/*
Sensor TOF to osc broadcast using microOSC
*/
byte ID = 1;
#include <Ethernet2.h>
EthernetUDP udp;

IPAddress broadcastIp(255, 255, 255, 255);
IPAddress sendIp(10, 0, 0, 255);
unsigned int sendPort = 7777;
unsigned int receivePort = 8888;


#include <MicroOscUdp.h>
// The number 1024 between the < > below  is the maximum number of bytes reserved for incomming messages.
// Outgoing messages are written directly to the output and do not need more reserved bytes.
MicroOscUdp<1024> oscUdp(&udp, sendIp, sendPort);


//  OSC MESSAGE
//Changer les derniers "chiffres de la mac addresse"
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xAE, ID };

//mac[5]=ID;
//
// ethPOE
#include <SPI.h>



#define SCK 22
#define MISO 23
#define MOSI 33
#define CS 19




// m5 LED
#include "M5Atom.h"

// couleur du led
#define RED 0xff, 0x00, 0x00
#define YELLOW 0xff, 0xff, 0x00
#define GREEN 0x00, 0xff, 0x00
#define CYAN 0x00, 0xff, 0xff
#define BLUE 0x00, 0x00, 0xff
#define PINK 0xff, 0x00, 0xff
#define OFF 0x00, 0x00, 0x00



uint8_t DisBuff[2 + 5 * 5 * 3];
void set_m5_led(uint8_t Rdata, uint8_t Gdata, uint8_t Bdata) {
  DisBuff[0] = 0x05;
  DisBuff[1] = 0x05;
  for (int i = 0; i < 25; i++) {
    DisBuff[2 + i * 3 + 1] = Rdata;  // -> fix : grb? weird?
    DisBuff[2 + i * 3 + 0] = Gdata;  //->  fix : grb? weird?
    DisBuff[2 + i * 3 + 2] = Bdata;
  }
  M5.dis.displaybuff(DisBuff);
}

void cycle_m5_color(int d_time) {
  delay(d_time);
  set_m5_led(RED);
  delay(d_time);
  set_m5_led(YELLOW);
  delay(d_time);
  set_m5_led(GREEN);
  delay(d_time);
  set_m5_led(CYAN);
  delay(d_time);
  set_m5_led(BLUE);
  delay(d_time);
  set_m5_led(PINK);
}


// \m5 LED

// TOF
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID 0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD 0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START 0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS 0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS 0x14
#define address 0x29  //I2C address

byte gbuf[16];

uint16_t bswap(byte b[]) {
  // Big Endian unsigned short to little endian unsigned short
  uint16_t val = ((b[0] << 8) & b[1]);
  return val;
}

uint16_t makeuint16(int lsb, int msb) {
  return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

void write_byte_data(byte data) {
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
}

void write_byte_data_at(byte reg, byte data) {
  // write data word at address and register
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void write_word_data_at(byte reg, uint16_t data) {
  // write data word at address and register
  byte b0 = (data & 0xFF);
  byte b1 = ((data >> 8) && 0xFF);

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(b0);
  Wire.write(b1);
  Wire.endTransmission();
}

byte read_byte_data() {
  Wire.requestFrom(address, 1);
  while (Wire.available() < 1) delay(1);
  byte b = Wire.read();
  return b;
}

byte read_byte_data_at(byte reg) {
  //write_byte_data((byte)0x00);
  write_byte_data(reg);
  Wire.requestFrom(address, 1);
  while (Wire.available() < 1) delay(1);
  byte b = Wire.read();
  return b;
}

uint16_t read_word_data_at(byte reg) {
  write_byte_data(reg);
  Wire.requestFrom(address, 2);
  while (Wire.available() < 2) delay(1);
  gbuf[0] = Wire.read();
  gbuf[1] = Wire.read();
  return bswap(gbuf);
}

void read_block_data_at(byte reg, int sz) {
  int i = 0;
  write_byte_data(reg);
  Wire.requestFrom(address, sz);
  for (i = 0; i < sz; i++) {
    while (Wire.available() < 1) delay(1);
    gbuf[i] = Wire.read();
  }
}

uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg) {
  // Converts the encoded VCSEL period register value into the real
  // period in PLL clocks
  uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
  return vcsel_period_pclks;
}

int read_tof_dist() {
  write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

  byte val = 0;
  int cnt = 0;
  while (cnt < 100) {  // 1 second waiting time max
    delay(10);
    val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
    if (val & 0x01) break;
    cnt++;
  }
  // if (val & 0x01) Serial.println("ready"); else Serial.println("not ready");

  read_block_data_at(0x14, 12);
  uint16_t acnt = makeuint16(gbuf[7], gbuf[6]);
  uint16_t scnt = makeuint16(gbuf[9], gbuf[8]);
  uint16_t dist = makeuint16(gbuf[11], gbuf[10]);
  byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);

  //Serial.print("-------\nambient count: "); Serial.println(acnt);
  //Serial.print("signal count: ");  Serial.println(scnt);
  //Serial.print("distance ");       Serial.println(dist);
  //Serial.print("status: ");        Serial.println(DeviceRangeStatusInternal);
  if (dist > 5000 || dist < 21) { dist = -1; }  //filter
  return dist;
}

// \end TOF


void setup() {
  // put your setup code here, to run once:
  M5.begin(true, false, true);
  Wire.begin(26, 32);  // join i2c bus (address optional for master)
  set_m5_led(RED);


  SPI.begin(SCK, MISO, MOSI, -1);
  Ethernet.init(CS);

  // init DHCP
  Ethernet.begin(mac);
  udp.begin(8888);
  set_m5_led(PINK);  // IF here; ethernet initialize


  // Create broadcast IP
  //ip_broadcast = Ethernet.localIP();
  //ip_broadcast[3]=255;

  // Print init state
  Serial.print("Button ip => ");
  Serial.print(Ethernet.localIP());
  Serial.print(" broadcasting => ");
  //Serial.println(ip_broadcast);


}


void loop() {
  // put your main code here, to run repeatedly:
  // listen for incoming clients
  M5.update();

  int distance = read_tof_dist();
  int distance_scaled=constrain(map(distance,20,2048,0,255),0,255);
  
  set_m5_led(255-distance_scaled,255-distance_scaled,10);

  if (distance > 0){
    oscUdp.sendMessage("/tof/raw","ii",ID,distance);
    oscUdp.sendMessage("/tof/scaled","ii",ID,distance_scaled);
  } else {
    // if out off boundaries, distance = -1;
  }
  Serial.print("/tof/raw ");
  Serial.println(distance);
  Serial.print("/tof/scaled ");
  Serial.println(distance_scaled);
  




  if (M5.Btn.wasPressed()) {
    Serial.println("BTN 1");
    oscUdp.sendMessage("/btn", "i", 1);
    cycle_m5_color(10);
  }
  if (M5.Btn.wasReleased()) {
    Serial.println("BTN 0");
    oscUdp.sendMessage("/btn", "i", 0);
    cycle_m5_color(10);
  }
}