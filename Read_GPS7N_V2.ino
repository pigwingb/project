/*

 29/08/2561     write Read_GPS7N_V1   i2c Master Writer/Slave Receiver
 06/09/2561     write Read_GPS7N_V2   https://www.arduino.cc/en/Tutorial/MasterReader
                                    data 16 byte lat 4 byte , lon 4 byte, _velocity_north 4 byte ,,_velocity_east 4 byte
 
support : Arduino 1.5.8   Arduino 328p 3.3V,
• GPS NEO-7N //

----------rx-----------  
A8 = PPM 8 CH
 */
#define I2C_GPS_ADDRESS                         8 //7 bits  0x20
#define I2C_GPS_LOCATION                        07   // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_NAV_LAT                         15   // Desired banking towards north/south int16_t
#define I2C_GPS_NAV_LON                         17   // Desired banking toward east/west    int16_t
#define I2C_GPS_WP_DISTANCE                     19   // Distance to current WP in cm uint32
#define I2C_GPS_WP_TARGET_BEARING               23   // bearing towards current wp 1deg = 1000 int16_t
#define I2C_GPS_NAV_BEARING                     25   // crosstrack corrected bearing towards current wp 1deg = 1000 int16_t
#define I2C_GPS_HOME_TO_COPTER_BEARING          27   // bearing from home to copter 1deg = 1000 int16_t
#define I2C_GPS_DISTANCE_TO_HOME                29   // distance to home in m int16_t 

#include <Arduino.h>
#include <Wire.h>
//#include <SoftwareSerial.h>
//SoftwareSerial swSer(12, 14);//14 ,12 (RXPin, TXPin); 
#include "Aconfigsam3x8e.h"
#include "GPSNEO8N_multi.h"

byte buf[16];//data lat 4 byte , lon 4 byte, _velocity_north 4 byte ,,_velocity_east 4 byte
////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Wire.begin(I2C_GPS_ADDRESS); //  // join i2c bus with address #8
  Wire.setClock(400000);
  Wire.onRequest(requestEvent); // register event
  Serial.begin(115200);//115200
  //Serial1.begin(57600);
  Serial.print("Read_GPSM8N_V1");Serial.println("\n");
  delay(200);
  GPS_multiInt();//"GPSNEO6N_multi.h"
  pinMode(13, OUTPUT);//pinMode (30, OUTPUT);pinMode (31, OUTPUT);//pinMode (30, OUTPUT);pinMode (31, OUTPUT);
  //(13=A=M),(31=B=STABLEPIN),(30=C,GPS FIG LEDPIN)
  //digitalWrite(2, HIGH);
  delay(100);
  //digitalWrite(2, LOW);
      for(uint8_t i=0;i<=10;i++){
      //GPS_NewData(); 
        while(Serial.available())
       {
       char byteGPS1=Serial.read(); 
       GPS_UBLOX_newFrame(byteGPS1);
       }
       GPS_LAT1 = GPS_coordLAT/10000000.0;// 1e-7 degrees / position as degrees (*10E7)
       GPS_LON1 = GPS_coordLON/10000000.0;
       GPS_LAT_HOME = GPS_LAT1;
       GPS_LON_HOME = GPS_LON1;
       digitalWrite(13, LOW);
       delay(20);
       digitalWrite(13, HIGH);
       delay(80);
    }
  digitalWrite(13, LOW);
  sensorPreviousTime = micros();
  previousTime = micros();
}
void loop() {
  while(1){
    while(Serial.available()){ /////GPS///////////////////////////////
     char byteGPS1=Serial.read(); 
     GPS_UBLOX_newFrame(byteGPS1);
     }//end gps  ///////////////////////////////////////
   Dt_roop = micros() - previousTime;// 200 Hz task loop (5 ms)  , 2500 us = 400 Hz
   if(Dt_roop <= 0){Dt_roop = 5001;}  
   if (Dt_roop >= 5000) 
    {
      previousTime = micros();
      G_Dt = Dt_roop*0.000001;
      frameCounter++;
  
        if (frameCounter % TASK_5HZ == 0)//GPS_calc TASK_5HZ
        {
           GPS_LAT1 = GPS_coordLAT/10000000.0;// 1e-7 degrees / position as degrees (*10E7)
           GPS_LON1 = GPS_coordLON/10000000.0;
           buf[0] = GPS_coordLAT & 255;
           buf[1] = (GPS_coordLAT >> 8)  & 255;
           buf[2] = (GPS_coordLAT >> 16) & 255;
           buf[3] = (GPS_coordLAT >> 24) & 255;
           buf[4] = GPS_coordLON & 255;
           buf[5] = (GPS_coordLON >> 8)  & 255;
           buf[6] = (GPS_coordLON >> 16) & 255;
           buf[7] = (GPS_coordLON >> 24) & 255;
           buf[8] = _velocity_north & 255;
           buf[9] = (_velocity_north >> 8)  & 255;
           buf[10] = (_velocity_north >> 16) & 255;
           buf[11] = (_velocity_north >> 24) & 255;
           buf[12] = _velocity_east & 255;
           buf[13] = (_velocity_east >> 8)  & 255;
           buf[14] = (_velocity_east >> 16) & 255;
           buf[15] = (_velocity_east >> 24) & 255;
           //Cal_GPS();//#include "Control_PPIDsam3x8e.h"
           //Control_PositionHold();
           //GPS_distance_m_bearing(GPS_LAT1, GPS_LON1, GPS_LAT_HOME, GPS_LON_HOME, Altitude_hat);
        }
        if (frameCounter % TASK_2HZ == 0)//LED GPS
        {
          if(Status_LED_GPS == LOW && GPS_FIX  == 1)
             {
               Status_LED_GPS = HIGH;
               Counter_LED_GPS++;
               if(Counter_LED_GPS >= GPS_numSat){
               Counter_LED_GPS = 0;
               digitalWrite(13, HIGH);
               }
               else
               {
               digitalWrite(13, LOW);
               }
             }
             else
             {
              Status_LED_GPS = LOW;
             }
            //digitalWrite(2, Status_LED_GPS);
        }//end if LED GPS

      if (frameCounter % TASK_5HZ == 0)//roop print  ,TASK_NoHZ TASK_5HZ  TASK_10HZ
        {
        }//end roop 5 Hz 
        
      if (frameCounter >= TASK_1HZ) { // Reset frameCounter back to 0 after reaching 100 (1s)
            frameCounter = 0;
            //temperaturetr = baro.getTemperature(MS561101BA_OSR_4096);
            //mpu6050_Temp_Values();
            //Remote_TrimACC();//motor.h
            if(Status_LED == LOW)
             {
              Status_LED = HIGH;
              //digitalWrite(13, LOW);
              }
            else
            {
            Status_LED = LOW;
            //digitalWrite(Pin_LED_G, HIGH);
            }
            //digitalWrite(13, Status_LED);//A Status_LED_GPS
            //digitalWrite(13, Status_LED_GPS);//A 
        }//end roop 1 Hz
    }//end roop 400 Hz
  }//end while roop
}
// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  //Wire.write("hello "); // respond with message of 6 bytes
  // as expected by master
 // Wire.beginTransmission(8); // transmit to device #8
  Wire.write(buf, sizeof(buf)); ////data 16 byte lat 4 byte , lon 4 byte, _velocity_north 4 byte ,,_velocity_east 4 byte
  //Wire.endTransmission();    // stop transmitting
}
