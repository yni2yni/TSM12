// ADS Touch Sensor Test Example Program (IC P/N:TSM12, 32QFN)
// Code:
// Date: 2021.02.15  Ver.: 0.1.3
// H/W Target: ARDUINO UNO R3, S/W: Arduino IDE  1.8.13
// Author: Park, Byoungbae (yni2yni@hanmail.net)
// Note: More information? Please send to e-mail.
// Uno R3, A4:SDA, A5: SCL, Leonardo 2:SDA,3:SCL, Uno R3, A4:SDA, A5: SCL,
// Register setting values are subject to change without prior notice to improve touch operation.

#include <Wire.h>

#define LF				0x0A //New Line
#define CR				0x0D //Carriage  return
#define SPC				0x20 //Spcae
                             
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

#define TSM12_SLAVE_GND  0x68 //8bit 0xD0<<1 //TSM12 ID Pin = GND
#define TSM12_SLAVE_VDD  0x78 //8bit 0xF0<<1 //TSM12 ID Pin = VDD

void  Init_TSM12(void); //Initialize TSM12

#define RESET_PIN 7 //Reset pin (H/W Reset Pin , or RC Reset Circuit)
#define EN_PIN    6 //I2C Enable Pin (TSM12 I2C Block Enable Pin, active Low)

void Register_Dump()
{
  byte read_data[1] = {0};

  for (int i = 0; i < 256; i += 16)
  {
    for (int j = 0; j <= 15; j++)
    {
      Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
      Wire.write((i + j));                   // sends register address
      Wire.endTransmission();                // stop transmitting
      Wire.requestFrom(TSM12_SLAVE_GND, 1);  // data read (2 byte)
      read_data[0] = Wire.read();            //
      print2hex(read_data, 1);               //
    }
    Serial.write(LF);
    Serial.write(CR);
  }
  delay(500);
}

void print2hex(byte *data, byte length) //Print Hex code
{
  Serial.print("0x");
  for (int i = 0; i < length; i++)
  {
    if (data[i] < 0x10)
    {
      Serial.print("0");
    }
    Serial.print(data[i], HEX);
    Serial.write(SPC);
  }
}

void setup(){

  delay(100); //wait for 100[msec]

  Wire.begin();        // join i2c bus (address optional for master)
  Wire.setClock(200000); // 200Khz

  Serial.begin(115200);  // start serial for output (Spped)
  // put your setup code here, to run once:
 
  pinMode(RESET_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

   // IC H/W reset signal,Active  High Reset
  digitalWrite(RESET_PIN, LOW); // Reset pin = low
  digitalWrite(RESET_PIN, HIGH); // Reset pin = Hihg
  delay(2); //Min: wait for 2[msec]
  digitalWrite(RESET_PIN, LOW); // Reset pin = low
  // IC H/W reset signal,Active  High Reset
  
  digitalWrite(EN_PIN, LOW); // TSM12 I2C Block Enable (Active Low)
  delay(100); //wait for 100[msec]

  Init_TSM12(); //Initialize TSM12
  
}
void loop() {

	byte read_data[3] ={0};

  //digitalWrite(EN_PIN, LOW); // TSM12 I2C Block Enable (Active Low)
  //delay(1);

  Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	Wire.write(byte(OUTPUT_REG1)); // sends register address
  Wire.endTransmission(); // stop transmitting
 	Wire.requestFrom(TSM12_SLAVE_GND,3); // read output data (3 byte)
  read_data[0]=Wire.read();
 	read_data[1]=Wire.read();
	read_data[2]=Wire.read();
  
  //digitalWrite(EN_PIN, HIGH); // TSM12 I2C Block Disable
  //delay(1);

  Serial.write(10);
  Serial.print("-------Touch Sensor Output Data  ---- > "); // Test Code

  print2hex(read_data, 3);
  //Serial.write(SP);
  //Serial.write(LF);
  //Serial.write(CR);

  delay(30);
  
}

void  Init_TSM12(void)
{
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL2)); // sends register address, 0x09h
   Wire.write(byte(0x0F)); // sends register data, S/W Reset Enable, Sleep Mode Enable
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity1)); // sends register address. 0x02h
   Wire.write(byte(0x33)); // sends register data
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity2)); // sends register address, 0x03h
   Wire.write(byte(0x33)); // sends register data
   Wire.endTransmission(); // stop transmitting
  
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity3)); // sends register address, 0x04h
   Wire.write(byte(0x33)); // sends register data
   Wire.endTransmission(); // stop transmitting

   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity4)); // sends register address, 0x05h
   Wire.write(byte(0x33)); // sends register data
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity5)); // sends register address, 0x06h
   Wire.write(byte(0x33)); // sends register data
   Wire.endTransmission(); // stop transmitting   
  
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity6)); // sends register address, 0x07h
   Wire.write(byte(0x33)); // sends register data
   Wire.endTransmission(); // stop transmitting   
   
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL1)); // sends register address, 0x08h
   Wire.write(byte(0x22));  
   // sends register data, Auto Moe,FTC=01, Interrupt(Middle,High), Response 4 (2+2)
   Wire.endTransmission(); // stop transmitting   
    
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Ref_rst1)); // sends register address, 0x0Ah
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Ref_rst2)); // sends register address, 0x0Bh
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting
  
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Ch_Hold1)); // sends register address, 0x0Ch
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting

   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Ch_Hold2)); // sends register address, 0x0Dh
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Cal_Hold1)); // sends register address, 0x0Eh
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting   
   
   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Cal_Hold2)); // sends register address, 0x0Fh
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting   

   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL2)); // sends register address, 0x09h
   Wire.write(byte(0x07)); // S/W Reset Disable, Sleep Mode Enable
   Wire.endTransmission(); // stop transmitting   
  
   
   }
// End 

