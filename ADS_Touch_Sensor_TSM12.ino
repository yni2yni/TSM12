// ADS Touch Sensor Test Example Program (IC P/N:TSM12)
// Code: 
// Date: 2016.04.28  Ver.: 0.1.0a
// H/W Target: ARDUINO UNO R3, S/W: Arduino IDE  1.6.8
// Author: Park, Byoungbae (yni2yni@hanmail.net)
// Note: More information? Please ,send to e-mail.
// Uno R3, A4:SDA, A5: SCL, Leonardo 2:SDA,3:SCL

#include <Wire.h>

#define AGS04_SLAVE_ADDR  0x6A //7bit:0xD4<<1
#define ANSG08_SLAVE_ADDR  0x24 //7bit:0x48<<1
#define ANSG08_OUTPUT	0x2A //cs1~cs4 output

#define Sensitivity1 	0x02 //ch2,ch1
#define Sensitivity2 	0x03 //ch4,ch3
#define Sensitivity3 	0x04 //ch6,ch5
#define Sensitivity4 	0x05 //ch8,ch7
#define Sensitivity5 	0x06 //ch10,ch9
#define Sensitivity6 	0x07 //ch12,ch11
#define CTRL1			0x08  
#define CTRL2			0x09
#define Ref_rst1		0x0A
#define Ref_rst2  		0x0B
#define Ch_Hold1	  	0x0C //Touch Key Channel Enable = 0x00
#define Ch_Hold2	  	0x0D //Touch Key Channel Enable = 0x00
#define Cal_Hold1	  	0x0E //Calibration Enable = 0x00
#define Cal_Hold2	  	0x0F //Calibration Enable = 0x00

#define OUTPUT_REG1  	0x10 //cs1~cs4 output
#define OUTPUT_REG2  	0x11 //cs5~cs8 output
#define OUTPUT_REG3  	0x12 //cs9~cs12 output

#define TSM12_SLAVE_GND  0x68 //0xD0<<1 //ID Pin = GND
#define TSM12_SLAVE_VDD  0x78 //0xF0<<1 //ID Pin = VDD

void  Init_TSM12(void); //Initialize TSM12

#define RESET_PIN 7 //Reset pin
#define EN_PIN    6 //I2C Enable Pin

void setup(){
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output (Spped)
  // put your setup code here, to run once:
 
  pinMode(RESET_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  
   // IC H/W reset signal,Active  High Reset
  digitalWrite(RESET_PIN, LOW);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(RESET_PIN, HIGH);
  delay(2); //Min: wait for 2[msec]
  digitalWrite(RESET_PIN, LOW);
  delay(100); //wait for 100[msec]
  //Init_TSM12(); //Initialize TSM12
  
}
void loop() {

	byte read_data;
  // put your main code here, to run repeatedly:
   Serial.println("--------Touch Sensor Output Data  ------ ");  // Test Code
   delay(5);
  
   Wire.beginTransmission(ANSG08_SLAVE_ADDR); // sned ic slave address
   Wire.write(byte(ANSG08_OUTPUT)); // sends register address
   Wire.endTransmission(); // stop transmitting
   Wire.requestFrom(ANSG08_SLAVE_ADDR,1); // read
   read_data=Wire.read();
   Serial.println(read_data); // send 
   
   //read_data[0]=Wire.read();
   //Serial.println(read_data[0],HEX); // send 
   //read_data[1]=Wire.read();
  // read_data[2]=Wire.read();
  // Serial.println(read_data[0],HEX); // send 
  // Serial.println(read_data[1],HEX);
  // Serial.println(read_data[2],HEX);
     
   delay(100);   
}

void  Init_TSM12(void)
{
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL2)); // sends register address
   Wire.write(byte(0x0F)); // sends register data
   Wire.endTransmission(); // stop transmitting

   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity1)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity2)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting
  
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity3)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting

   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity4)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity5)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting   
  
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity6)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting   
   
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL1)); // sends register address
   Wire.write(byte(0x22A)); // sends register data
   Wire.endTransmission(); // stop transmitting    

   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Ref_rst1)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Ref_rst2)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting
  
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Ch_Hold1)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting

   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Ch_Hold2)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Cal_Hold1)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting   
   
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Cal_Hold2)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting   

   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL2)); // sends register address
   Wire.write(byte(0x03)); // sends register data
   Wire.endTransmission(); // stop transmitting   
  
   }
// End 
