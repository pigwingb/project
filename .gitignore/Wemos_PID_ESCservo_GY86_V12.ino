/*
 8/5/2561   Wemos_PID_ESCservo_GY86_V1  ,, read gy86 ,write servo and esc , read PPM
 9/5/2561   Wemos_PID_ESCservo_GY86_V2  ,, control PID
 10/5/2561  Wemos_PID_ESCservo_GY86_V3  ,, refresh servos 100 Hz =  10000 microseconds
 13/5/2561  Wemos_PID_ESCservo_GY86_V4  ,, servo ES315 ,,
 13/5/2561  Wemos_PID_ESCservo_GY86_V5  ,, kain PID, roll, pitch, yaw ,,pitch_I_rate
 14/8/2561  Wemos_PID_ESCservo_GY86_V6  ,, new frame 
 23/8/2561  Wemos_PID_ESCservo_GY86_V7  ,, new frame ,,new kain PID ,, servo_trim_L,R
 06/9/2561  Wemos_PID_ESCservo_GY86_V8  ,, read i2c arduino328p gps ,,Wemos_udpFromPhone_V3
 25/9/2561  Wemos_PID_ESCservo_GY86_V9  ,, Read HMC5883L ,, Pressure MS5611
 30/9/2561  Wemos_PID_ESCservo_GY86_V10 ,, Observer_kalman Altitude Hold ,,PID GPS Position Hold
 06/10/2561 Wemos_PID_ESCservo_GY86_V11 ,, Baro xx
 10/10/2561 Wemos_PID_ESCservo_GY86_V12 ,, Auto
  
support: 
• Arduino 1.8.5
• ESP8266 wemos D1 mini pro
http://arduino.esp8266.com/stable/package_esp8266com_index.json 
C:\Users\SAMd\AppData\Roaming\Arduino15\packages\esp8266\hardware\esp8266\2.4.2\libraries\Servo

• GY86 ,MPU6050 6 axis gyro/accel with Motion Processing Unit
• Bicopter 2 prop 2 servo 380 mm
• i2c pin D2=SDA D1=SCL begin(int sda, int scl);
• LED pin D0 ,GPIO-16

--------Type Bi-Copter Frame-------------      
         6  12>>         <<14  5

        servo                servo
        8  15>>             <<13  7
----------rx-----------         
PPM pin D3  
Wifi ,,ppm  => 0 , Key   ,12345678 , 12345      
*/
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "MS561101BA.h"
#include "config.h"
#include "ahrs.h"
#include "mpu6050.h"
//#include "ahrs.h"
#include "Kalman_Observer.h"
#include "remotePPM.h"
#include "Control_PGPS.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

/* ----- Set AP ---------- */
const char *ssid = "ESP8266_GPS_AP";
const char *password = "12345679";
/* ---- End set AP --------*/
unsigned int localPort = 5000;
IPAddress local_ip = {192,168,4,2};
IPAddress gateway = {192,168,4,1};
IPAddress subnet = {255,255,255,0};
//_____________Set UDP variables_____________//
char packetBuffer[255]; // char buffer to hold incoming packet

float lat = 13.868172f;//13.8695810 100.478444
float lon = 100.49655f;
float alt = 0.1f;
float Start_Lat = 13.868172f;
float Start_Lon = 100.49655f;
float Start_Alt = 0.0f;
float End_Lat = 13.868172f;
float End_Lon = 100.49655f;
float End_Alt = 0.0f;

String b,ct;
byte ij=0;
char byteSend[4];
//_____________End set UDP Variables_________//
//Convert float to byte////////////////////////////
union{
 float number1;
 byte bytes1[4];
} myFloat1;
union{
 float number2;
 byte bytes2[4];
} myFloat2;
union{
 float number3;
 byte bytes3[4];
} myFloat3;
union{
 float number4;
 byte bytes4[4];
} myFloat4;
//////////////////////////////////////////////
WiFiUDP Udp;
//#define DEFAULT_PULSE_WIDTH  1000     // default pulse width when servo is attached
//#define REFRESH_INTERVAL    10000     // minumim time to refresh servos in microseconds 
//#define SERVOS_PER_TIMER       8     // the maximum number of servos controlled by one timer 
#define I2C_GPS_ADDRESS    8 //7 bits  0x20

Servo escR5,escL6;
Servo serR7,serL8;
void setup()
{
 // initialize the serial link with processing
  Serial.begin(115200);
  delay(200);
  WiFi.softAP(ssid, password);
  Udp.begin(localPort);
  //End open wifi
  WiFi.config(local_ip,gateway,subnet);
  Serial.println(" ");
  Serial.println("Wemos_udpFromPhone_V3.ino ");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("UDP Port: ");
  Serial.println(localPort);
  Serial.println("Wemos_PID_ESCservo_GY86_GPS_V12.ino ");
  escR5.attach(14);
  escL6.attach(12);
  serR7.attach(13);
  serL8.attach(15);
  escR5.writeMicroseconds(1000);
  escL6.writeMicroseconds(1000);
  serR7.writeMicroseconds(servo_trim_R);
  serL8.writeMicroseconds(servo_trim_L);
  Serial.println("remotePPM_initialize() ");
  remotePPM_initialize();
  Wire.begin(D2,D1);//begin(int sda, int scl);
  Wire.setClock(400000);
  delay(200);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(200);
    mpu6050_initialize();
    Serial.println("mpu6050_initialize() ");
    delay(10);
    MagHMC5883Int();
    Serial.println("MagHMC5883Int() ");
    delay(10);
    ahrs_initialize();  
    Serial.println("ahrs_initialize() ");     
    delay(10); 
    baro.init(MS561101BA_ADDR_CSB_LOW);
    Serial.println("MS561101BA::init() ");     
    delay(10); 
    Serial.println("mpu6050_Calibrate() "); 
    mpu6050_Gyro_Calibrate();
    mpu6050_Accel_Calibrate();
    delay(10);
    sea_press = presser + 0.11;//presser 1003.52
    Serial.println(sea_press); 
    previousTime = micros(); 
    sensorPreviousTime = previousTime;
    hundredHZpreviousTime = previousTime;
}
float getAltitude(float pressure2, float temperature2)
{
  //return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
  return log(sea_press/pressure2) * (temperature2+273.15) * 29.271267f; // in meter 
  //return ((pow((sea_press/pressure),1/5.257)-1.0)*(temperature+273.15))/0.0065;
}
void loop()
{
    // Timer
    currentTime = micros();
    readUDP();
    // Read data (not faster then every 1 ms) // 1000 Hz
    if (currentTime - sensorPreviousTime >= 1000) 
    {
        mpu6050_readGyroSum();
        mpu6050_readAccelSum(); 
        sensorPreviousTime = currentTime;
    }
    // 100 Hz task loop (10 ms),, ,250 Hz = 4000 us ,,,, 400 Hz = 2500 us
    if (currentTime - hundredHZpreviousTime >= 10000) //10000
    {
        frameCounter++;
        G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
        hundredHZpreviousTime = currentTime; 
        mpu6050_Get_gyro();
        mpu6050_Get_accel();
        presser = baro.getPressure(MS561101BA_OSR_4096);//read 100 Hz
        temperature = baro.getTemperature(MS561101BA_OSR_4096);
        Altitude_baro = getAltitude(presser,temperature);//Altitude_Ground
        Altitude_barof = Altitude_baro - Altitude_baroEarth;
   ////////////////Moving Average Filters///////////////////////////
      GyroXf = (gyro[XAXIS] + GyroX2)/2.0;
      GyroYf = (gyro[YAXIS] + GyroY2)/2.0;
      GyroZf = (gyro[ZAXIS] + GyroZ2)/2.0;
      GyroX2 = gyro[XAXIS];GyroY2 = gyro[YAXIS];GyroZ2 = gyro[ZAXIS];//gyro Old1
        ////////////////Low pass filter/////////////////////////////////
      //GyroZf = GyroZf + (GyroZ - GyroZf)*49.6*G_Dt;
    AccXf = AccXf + (accel[XAXIS] - AccXf)*0.121;//12.4  //Low pass filter ,smoothing factor  α := dt / (RC + dt)
    AccYf = AccYf + (accel[YAXIS] - AccYf)*0.121;//12.4
    AccZf = AccZf + (accel[ZAXIS] - AccZf)*0.121;//12.4
/////////////////////////////////////////////////////////////////////////
        //ahrs_updateIMU(GyroXf,GyroYf,GyroZf, AccXf,AccYf,AccZf,G_Dt);
        ahrs_updateMARG(GyroXf, GyroYf, GyroZf, AccXf, AccYf, AccZf, c_magnetom_x, c_magnetom_y, c_magnetom_z, G_Dt);//quaternion ,direction cosine matrix ,Euler angles
        Observer_kalman_filter();
////PID Control////////////////////////////////////////////////////////////////////
   // YAW CONTROL
        err_yaw_rate = ((CH_RUD - 1500)*0.0035) - gyro[ZAXIS];//0.0055  
        yaw_I_rate += err_yaw_rate*Ki_rateYaw*G_Dt;     
        yaw_I_rate = constrain(yaw_I_rate, -150, 150);        
        yaw_D_rate = (err_yaw_rate-err_yaw_ant_rate)/G_Dt;    
        err_yaw_ant_rate = err_yaw_rate;         
        control_yaw = (Kp_rateYaw*err_yaw_rate) + yaw_I_rate + (Kd_rateYaw*yaw_D_rate);
        control_yaw = constrain(control_yaw,-300,300); 
   // ROLL CONTROL angle
            err_roll_level = (((CH_AIL - 1500)*(0.0011)) - ahrs_r);//0.0015
   if(Mode == 2 || Mode == 3){
    err_roll_level = (Control_YBf*0.0174532925) - ahrs_r;
  }
            roll_I_level += err_roll_level*Ki_levelRoll*G_Dt; 
            roll_I_level = constrain(roll_I_level, -2, 2); 
            roll_D_level = (err_roll_level-err_roll_ant_level)/G_Dt; 
            err_roll_ant_level = err_roll_level;        
            control_roll_angle = Kp_levelRoll*err_roll_level + roll_I_level + Kd_levelRoll*roll_D_level; 
            control_roll_angle = constrain(control_roll_angle, -35, 35);
  // PITCH CONTROL angle
            err_pitch_level = (((CH_ELE - 1500)*(-0.0011)) - ahrs_p);//-0.0011  
   if(Mode == 2 || Mode == 3){
    err_pitch_level = (Control_XBf*0.0174532925) - ahrs_p;
  }
            pitch_I_level += err_pitch_level*Ki_levelPitch*G_Dt;  
            pitch_I_level = constrain(pitch_I_level, -2, 2);
            pitch_D_level = (err_pitch_level-err_pitch_ant_level)/G_Dt;       
            err_pitch_ant_level = err_pitch_level;   
            control_pitch_angle = Kp_levelPitch*err_pitch_level + pitch_I_level + Kd_levelPitch*pitch_D_level;   
            control_pitch_angle = constrain(control_pitch_angle, -35, 35);
   //ROLL CONTROL Gyro
            err_roll_rate = control_roll_angle - (gyro[XAXIS]);  
            roll_I_rate += err_roll_rate*Ki_rateRoll*G_Dt;     
            roll_I_rate = constrain(roll_I_rate, -10, 10);            
            roll_D_rate = (err_roll_rate-err_roll_ant_rate)/G_Dt;
            err_roll_ant_rate = err_roll_rate;             
            control_roll = (Kp_rateRoll*err_roll_rate) + roll_I_rate + (Kd_rateRoll*roll_D_rate);
            control_roll = constrain(control_roll,-300,300); 
  //PITCH CONTROL Gyro
            err_pitch_rate = control_pitch_angle - gyro[YAXIS];  
            pitch_I_rate += err_pitch_rate*Ki_ratePitch*G_Dt;     
            pitch_I_rate = constrain(pitch_I_rate, -300, 300);
            pitch_D_rate = (err_pitch_rate-err_pitch_ant_rate)/G_Dt;
            err_pitch_ant_rate = err_pitch_rate; 
            control_pitch = (Kp_ratePitch*err_pitch_rate) + pitch_I_rate + (Kd_ratePitch*pitch_D_rate);
            control_pitch = constrain(control_pitch,-500,500);
            Control_PPIDRate();
/////mix Bi-Copter Frame//////////////////////////////////////////////////////////////////////////////
 if (armed == 1)
        {
            if (CH_THR < 1050) 
            {
            motor_FRONTL = 1060;
            motor_FRONTR = 1060;
            servo_REARL = servo_trim_L;
            servo_REARR = servo_trim_R; 
                roll_I_rate=0.0f;  
                pitch_I_rate=0.0f;
                yaw_I_rate=0.0f;
                roll_I_level=0.0f;
                pitch_I_level=0.0f;
            }else{
                //Bi-Copter
                motor_FRONTL = constrain((uAltitude + control_roll), 1050, 1950);
                motor_FRONTR = constrain((uAltitude - control_roll), 1050, 1950);
                servo_REARL = constrain((servo_trim_L - control_pitch + control_yaw), 900, 2200);
                servo_REARR = constrain((servo_trim_R + control_pitch + control_yaw), 900, 2200);           
            }              
        }
////////////////////////////////////////
         if (armed == 0) 
        {
            motor_FRONTL = 1000;
            motor_FRONTR = 1000;
            servo_REARL = constrain((servo_trim_L - control_pitch + control_yaw), 900, 2200);
            servo_REARR = constrain((servo_trim_R + control_pitch + control_yaw), 900, 2200);  
            //servo_REARL = AUX_2;//servo_trim_L
            //servo_REARR = AUX_3;//servo_trim_R
            roll_I_rate=0.0f;  
            //pitch_I_rate=0.0f;
            yaw_I_rate=0.0f;
            roll_I_level=0.0f;
            //pitch_I_level=0.0f;  
        }
////out servo///command Motors///////////////////////////////////////////////////////////////////
  escR5.writeMicroseconds(motor_FRONTR);
  escL6.writeMicroseconds(motor_FRONTL);
  serR7.writeMicroseconds(servo_REARR);
  serL8.writeMicroseconds(servo_REARL);
//////////////////////////////////////////////////////////////////////////
        // 50 Hz task (20 ms)
        if (frameCounter % TASK_50HZ == 0) 
        {
          computeRC();
          Chack_Command();
            if (CH_THR < 1080) 
        {
            if (CH_RUD > 1700 && armed == 0) //armed
            {
                armed = 1;
                Altitude_baroEarth = Altitude_baro;
            }
            
            if (CH_RUD < 1200 && armed == 1) //not armed
            {    
                armed = 0;
    
            }
        }
      }
      if (frameCounter % TASK_20HZ == 0)// 20 Hz task (50 ms)
        {
          Control_PositionHold();//#include "Control_PID.h"
        }
////////////end 20 Hz tak///////////////////////////////////////////////////////////////////// 
        // 10 Hz task (100 ms)//////////////////////////////////////////////////
        if (frameCounter % TASK_10HZ == 0) 
        {
         Mag5883Read();
         Vz_Baro_ult = (Altitude_baro - Vz_Baro_ultold)/0.1;//diff Altitude
         Vz_Baro_ultold = Altitude_baro;
         Automatictakeland();
        }
       // 5 Hz task (200 ms) Read GPS///////////////////////////////////////////
        if (frameCounter % TASK_5HZ == 0) 
        {
          Wire.requestFrom(I2C_GPS_ADDRESS, 16);//data 16 byte lat 4 byte , lon 4 byte, _velocity_north 4 byte ,,_velocity_east 4 byte
          buf1[0] = Wire.read();
          buf1[1] = Wire.read();
          buf1[2] = Wire.read();
          buf1[3] = Wire.read();
          buf1[4] = Wire.read();
          buf1[5] = Wire.read();
          buf1[6] = Wire.read();
          buf1[7] = Wire.read();
          buf1[8] = Wire.read();
          buf1[9] = Wire.read();
          buf1[10] = Wire.read();
          buf1[11] = Wire.read();
          buf1[12] = Wire.read();
          buf1[13] = Wire.read();
          buf1[14] = Wire.read();
          buf1[15] = Wire.read();
          Wire.endTransmission();
          GPS_degree[0] = ((buf1[3] << 24) | (buf1[2] << 16) | (buf1[1] << 8) | buf1[0]);
          GPS_degree[1] = ((buf1[7] << 24) | (buf1[6] << 16) | (buf1[5] << 8) | buf1[4]);
          _velocity_north = ((buf1[11] << 24) | (buf1[10] << 16) | (buf1[9] << 8) | buf1[8]);
          _velocity_east = ((buf1[15] << 24) | (buf1[14] << 16) | (buf1[13] << 8) | buf1[12]);
      }//end 5 Hz
        // 10 Hz task (100 ms)//////////////////////////////////////////////////////////////////////
        if (frameCounter % TASK_10HZ == 0) 
        {
            //G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
            //lowPriorityTenHZpreviousTime = currentTime;
            
//            //x
            //Serial.print(motor_FRONTL);Serial.print("\t");
            //Serial.print(motor_FRONTR);Serial.print("\t");
            //Serial.print(motor_REARL);Serial.print("\t");    
            //Serial.print(motor_REARR);Serial.print("\t");

//            Serial.print(r_KiTerm);Serial.print("\t");    
//            Serial.print(r_dInput);Serial.print("\t");     
//            Serial.print(r_value);Serial.print("\t");     
        
//            Serial.print(control_roll);Serial.print("\t");    
//            Serial.print(control_pitch);Serial.print("\t");     
//            Serial.print(control_yaw);Serial.print("\t");       

            //Serial.print(CH_THR);Serial.print("\t");
            //Serial.print(CH_AIL);Serial.print("\t");  
            //Serial.print(CH_ELE);Serial.print("\t");
            //Serial.print(CH_RUD);Serial.print("\t");  
            //Serial.print(AUX_1);Serial.print("\t"); 
            //Serial.print(AUX_2);Serial.print("\t");
            //Serial.print(AUX_3);Serial.print("\t");
            
            //Serial.print(gyro[XAXIS]*180/PI);Serial.print("\t");
            //Serial.print(gyro[YAXIS]*180/PI);Serial.print("\t");
            //Serial.print(gyro[ZAXIS]*180/PI);Serial.print("\t");
        
            //Serial.print(accel[XAXIS]);Serial.print("\t");
            //Serial.print(accel[YAXIS]);Serial.print("\t");
            //Serial.print(accel[ZAXIS]);Serial.print("\t");
            
            //Serial.print(MagXf);Serial.print("\t");
            //Serial.print(MagYf);Serial.print("\t");
            //Serial.print(MagZf);Serial.print("\t");

            //Serial.print(temperature);Serial.print("\t");
            //Serial.print(presser);Serial.print("\t");
            //Serial.print(Altitude_baro);Serial.print("\t");
            Serial.print(z1_hat);Serial.print("\t");
            //Serial.print(z2_hat);Serial.print("\t");
            //Serial.print(z3_hat);Serial.print("\t");
            
            //Serial.print(ahrs_r*180/PI);Serial.print("\t");
            //Serial.print(ahrs_p*180/PI);Serial.print("\t");
            //Serial.print(ahrs_y*180/PI);Serial.print("\t");

//            Serial.print(ahrs_r);Serial.print("\t");
//            Serial.print(ahrs_p);Serial.print("\t");
//            Serial.print(ahrs_y);Serial.print("\t");

//            Serial.print(accel_offset[XAXIS]);Serial.print("\t");
//            Serial.print(accel_offset[YAXIS]);Serial.print("\t");
//            Serial.print(accel_offset[ZAXIS]);Serial.print("\t");
              //Serial.print(accrZ_Earth);Serial.print("\t");
              
            //Serial.print(GPS_degree[0]);Serial.print("\t");
            //Serial.print(GPS_degree[1]);Serial.print("\t");
            //Serial.print(_velocity_north);Serial.print("\t");
            //Serial.print(_velocity_east);Serial.print("\t");
            Serial.print(x1_hat,8);Serial.print("\t");
            Serial.print(y1_hat,8);Serial.print("\t");
            
       //Serial.print(Start_Lat,8);Serial.print("\t");
       //Serial.print(Start_Lon,8);Serial.print("\t");
       // Serial.print(Start_Alt);Serial.print("\t");
       //Serial.print(End_Lat,8);Serial.print("\t");
       //Serial.print(End_Lon,8);Serial.print("\t");
       // Serial.print(End_Alt);Serial.print("\t");

       Serial.print(Control_XBf);Serial.print("\t");
       Serial.print(Control_YBf);Serial.print("\t");
            Serial.print(1/G_Dt);Serial.print("\t");
            Serial.println("");            
        }  
        // Reset frameCounter back to 0 after reaching 100 (1s)
        if (frameCounter >= TASK_1HZ) {
            frameCounter = 0;
            blink1();
            time_auto = time_auto + 1;
        }
        
        //previousTime = currentTime;
    } //250 Hz
}
void blink1(void)
{
  static int8_t led = 0;
  led = 1 - led;
  digitalWrite(LED_BUILTIN, led);
}
// function get data from UPD
void readUDP() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize > 0 )  // if have data
  {
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
      b = packetBuffer;
      //Serial.print(len);Serial.print("\t");
      //Serial.print(packetBuffer[0],HEX);Serial.print("\t");
      //Serial.print(packetBuffer[1],HEX);Serial.print("\t");
      //Serial.print(packetBuffer[2],HEX);Serial.print("\t");
      //Serial.print(packetBuffer[3],HEX);Serial.print("\t");
      //Serial.print(alt);Serial.print("\t");
      //Serial.println(b);
      if(len == 24){
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.print("REQ");
        Udp.endPacket();
        myFloat4.bytes4[0] = packetBuffer[0];
        myFloat4.bytes4[1] = packetBuffer[1];
        myFloat4.bytes4[2] = packetBuffer[2];
        myFloat4.bytes4[3] = packetBuffer[3];
      Start_Lat = myFloat4.number4;
        myFloat4.bytes4[0] = packetBuffer[4];
        myFloat4.bytes4[1] = packetBuffer[5];
        myFloat4.bytes4[2] = packetBuffer[6];
        myFloat4.bytes4[3] = packetBuffer[7];
      Start_Lon = myFloat4.number4;
        myFloat4.bytes4[0] = packetBuffer[8];
        myFloat4.bytes4[1] = packetBuffer[9];
        myFloat4.bytes4[2] = packetBuffer[10];
        myFloat4.bytes4[3] = packetBuffer[11];
      Start_Alt = myFloat4.number4;
        myFloat4.bytes4[0] = packetBuffer[12];
        myFloat4.bytes4[1] = packetBuffer[13];
        myFloat4.bytes4[2] = packetBuffer[14];
        myFloat4.bytes4[3] = packetBuffer[15];
      End_Lat = myFloat4.number4;
        myFloat4.bytes4[0] = packetBuffer[16];
        myFloat4.bytes4[1] = packetBuffer[17];
        myFloat4.bytes4[2] = packetBuffer[18];
        myFloat4.bytes4[3] = packetBuffer[19];
      End_Lon = myFloat4.number4;
        myFloat4.bytes4[0] = packetBuffer[20];
        myFloat4.bytes4[1] = packetBuffer[21];
        myFloat4.bytes4[2] = packetBuffer[22];
        myFloat4.bytes4[3] = packetBuffer[23];
      End_Alt = myFloat4.number4;
      GPS_LAT_HOME = Start_Lat;
      GPS_LON_HOME = Start_Lon;
      target_LAT = x1_hat;
      target_LON = y1_hat;
      waypoint1_LAT = End_Lat;
      waypoint1_LON = End_Lon;
        //Serial.println(len);
      }
      if(b == "REQ"){
        //test ++ data
        lat = GPS_degree[0]/10000000.0;// 1e-7 degrees / position as degrees (*10E7);
        lon = GPS_degree[1]/10000000.0;// 1e-7 degrees / position as degrees (*10E7);
        alt = z1_hat + 0.2;
          myFloat1.number1 = lat;
          myFloat2.number2 = lon;
          myFloat3.number3 = alt;    
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.write(myFloat1.bytes1, 4);
          Udp.write(myFloat2.bytes2, 4);
          Udp.write(myFloat3.bytes3, 4);
          Udp.endPacket();
      }
    }
  }
}
// end function get data from UPD
