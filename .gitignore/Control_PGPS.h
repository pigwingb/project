/*
• Arduino 1.8.5
• ESP8266 wemos D1 mini pro
http://arduino.esp8266.com/stable/package_esp8266com_index.json 
C:\Users\SAMd\AppData\Roaming\Arduino15\packages\esp8266\hardware\esp8266\2.4.2\libraries\Servo

• GY86 ,MPU6050 6 axis gyro/accel with Motion Processing Unit
• Bicopter 2 prop 2 servo 380 mm
*/
void Control_PPIDRate(){
 ////Altitude//////////////////////////////////////////////////////////////////////////////////////////////////////  
 Velocity_THR = (CH_THR - 1500.0f)*0.00369f;//0.00469 +- 2.1 m/s
 applyDeadband(Velocity_THR, 0.23f);//0.23 0.17
  if(Mode == 1 || Mode == 2)//Altitude Hold, 
  {
    Altitude_Hold = Altitude_Hold + (Velocity_THR*G_Dt);//Integral Velocity_THR
    Altitude_Hold = constrain(Altitude_Hold, 0.0f, Altitude_Max);//0 - 10 m
    err_hz = Altitude_Hold - z1_hat;
    err_hz = constrain(err_hz, -1.0f, 1.0f);//+-2 m
    float error_Vz = Velocity_THR*0.3015f - z2_hat;//0.32 0.55
    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
    error_Vz_old = error_Vz;
   // float error_hII = err_hz*1.25 + error_Vz;//0.55
    hz_I = hz_I + (Ki_altitude*err_hz*G_Dt);
    hz_I = constrain(hz_I, -100.0f, 600.0f);//+-600
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (z3_hat*Ka_altitude);//state feedback
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
  }
  else if(Mode == 3)//Automatic  Takeoff, Landing
  {
    err_hz = h_counter - z1_hat;
    err_hz = constrain(err_hz, -1.0f, 1.0f);//+-2 m
    float error_Vz = Vz_Hold - z2_hat;//Vz_Hold 
    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
    error_Vz_old = error_Vz;
    //float error_hII = err_hz*1.25 + error_Vz;//0.55
    hz_I = hz_I + (Ki_altitude*err_hz*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -100.0f, 600.0f);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (z3_hat*Ka_altitude);//state feedback
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
  }
  else
  {
    hz_I = 0.0f;
    uthrottle = 0.0f;
    Altitude_Hold = z1_hat;
    I_throttle = CH_THR;
  }//end Altitude Hold
//uthrottle = constrain(uthrottle, -200, 200);//+-120 +-150
uAltitude = (I_throttle + uthrottle);//m*g = 10.8 N = 
uAltitude = constrain(uAltitude, 1050, 1950);
}
void Chack_Command(){
   if(AUX_1 <= (AltHold-10))//Stabilize 
  {
    Mode = 0;
  }
   if(AUX_1 > (AltHold-10) && AUX_1 <= (AltHold+10))//Altitude Hold, 
  {
    Mode = 1;
  }
   if(AUX_1 > (PositionHold-10) && AUX_1 <= (PositionHold+10))//Position Hold
  {
    Mode = 2;
  }  
  if(AUX_1 > (Auto-10))//Automatic  Takeoff
  {
    Mode = 3;
  }
  if(AUX_1 > (RTH-10) && AUX_1 <= (RTH+10))//RTH
  {
   //Mode = 2;
   //target_LAT = GPS_LAT_HOME;//GPS_LAT_Hold
   //target_LON = GPS_LON_HOME;//GPS_LON_Hold
  }
  //////////////////
   if(AUX_2 <= 1300)//Set Home 
  {
    GPS_LAT_HOME = x1_hat;
    GPS_LON_HOME = y1_hat;
    //digitalWrite(Pin_LED_G, HIGH);
    //digitalWrite(Pin_LED_R, LOW);
  }
   if(AUX_2 >= 1700)//Set waypoint1  
  {
    waypoint1_LAT = x1_hat;
    waypoint1_LON = y1_hat;
    //digitalWrite(Pin_LED_R, HIGH);
    //digitalWrite(Pin_LED_G, LOW);
  }
}//end Chack_Command()
///////////////////////////////////////////////////////////////////////////////////
void Automatictakeland(){
 //Altitude control and 1 waypoint navigation
  if(Mode == 3 && CH_THR > 1100 && armed == 1)
  {
    if(time_auto < 2){//Check time < 5
      takeoff = 1;
    }
     if(z1_hat >= h_control && endAuto == 1)//waypoint1
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint = 1;
    }
     if(time_auto > 8 && abs(error_LAT) <= 70.0f && abs(error_LON) <= 70.0f && endAuto == 1 && Status_waypoint == 1)//50 10 Landing and position hold mode
    {
      timeLanding++;
      if(timeLanding >= 30)//relay 3 s Landing
      {
       takeoff = 0;
      }
    }
  }
  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1)
  {
    takeoff = 0;
    timeLanding = 0;
    timeOff = 0;
    time_auto = 0;
    h_counter = 0.1f;//0.0
    h_counter_old = 0.1f;
    Vz_Hold = 0.0f;
    Status_waypoint = 0;
  } 
////////////////////////////////////////////////////////////////// 
       if(h_counter < h_control && takeoff == 1)//take-off
      {
        endAuto = 1;
        h_counter = h_counter + 0.042f;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold = (h_counter - h_counter_old)/0.100050025f;//(m/s) roop 10 Hz
        h_counter_old = h_counter;
      }
       if(takeoff == 0 && endAuto == 1)//landing
      {
        h_counter = h_counter - 0.029f;//0.023 ramp input hz  landing
        //Vz_Hold = (h_counter - h_counter_old)/0.100050025f;//(m/s) roop 10 Hz
        Vz_Hold = -0.18f;
        //h_counter_old = h_counter;
         if(z1_hat <= 0.3f)
        {
          Vz_Hold = 0.0f;
        }
        if(z1_hat <= 0.18f)
        {
         endAuto = 0;
        }
      }
////////////////////////////////////
      if(time_auto > 25 && endAuto == 0) //15 s End waypoint quadrotor
        {
          timeOff++;
          if(timeOff > 60)//relay 6 s time-Off
          {
           armed = 0;
          } 
        }  
///////////////////////////////////////////////////////////
}
///////////////////////////////////////////////////////////////////////////////
void Control_PositionHold(){
  if(Mode == 2 || Mode == 3){
          float targetRB_speedLAT = ((CH_ELE-1500)*0.624f);//vX Body Frame ,,0.444 = 200 cm/s
          float targetRB_speedLON = ((CH_AIL-1500)*0.624f);//vY Body Frame
          applyDeadband(targetRB_speedLAT, 9.20f);//22.22 ,, +-50
          applyDeadband(targetRB_speedLON, 9.20f);//22.22
          targetRE_speedLAT = (targetRB_speedLAT*DCM00 + targetRB_speedLON*DCM01);//vX Earth Frame
          targetRE_speedLON = (targetRB_speedLAT*DCM10 + targetRB_speedLON*DCM11);//vY Earth Frame
/////////////////////////////////////////////////////////////////////////////////////////////////////
          target_LAT = target_LAT + ((double)targetRE_speedLAT*0.0498753d/cm_per_deg_lat);//roop 20 Hz = 0.0498753 s
          target_LON = target_LON + ((double)targetRE_speedLON*0.0498753d/cm_per_deg_lon);//roop 20 Hz = 0.0498753 s
    ////////////////////////////////////////////////////////////////////////////////////////////
          error_LAT = (double)(target_LAT - x1_hat)*cm_per_deg_lat; //*10939761.4 X Error cm
          error_LON = (double)(target_LON - y1_hat)*cm_per_deg_lon;//*6371000.0 Y Error   cm
          error_LAT = constrain(error_LAT,-500.0f,500.0f);//200 = +-2 m
          error_LON = constrain(error_LON,-500.0f,500.0f);
          float target_speedLAT = (error_LAT*Kp_speed) + (targetRE_speedLAT*0.312f);//P Control Velocity GPS //+-200 cm/s = 2m/s
          float target_speedLON = (error_LON*Kp_speed) + (targetRE_speedLON*0.312f);//P Control Velocity GPS
          //target_speedLAT = constrain(target_speedLAT,-200,200);//+-200 cm/s = 2m/s
          //target_speedLON = constrain(target_speedLON,-200,200);

          float error_rate_LAT = target_speedLAT - vx2_hat;//m/s to cm/s
          float error_rate_LON = target_speedLON - vy2_hat;

          error_rate_LAT = constrain(error_rate_LAT,-300.0f,300.0f);//+-300 cm/s ,200
          error_rate_LON = constrain(error_rate_LON,-300.0f,300.0f);
          GPSI_LAT = GPSI_LAT + (error_rate_LAT*Ki_gps*0.05f);//5Hz=0.2 ,, 20 Hz = 0.05
          GPSI_LON = GPSI_LON + (error_rate_LON*Ki_gps*0.05f);  
          GPSI_LAT = constrain(GPSI_LAT,-300.0f,300.0f);//win speed +-200 cm/s
          GPSI_LON = constrain(GPSI_LON,-300.0f,300.0f);//300
          //Control_XEf = error_rate_LAT*Kd_gps;//P Control speed 
          //Control_YEf = error_rate_LON*Kd_gps;
          Control_XEf = error_LAT*Kp_gps + error_rate_LAT*Kd_gps + GPSI_LAT;//PID Control speed 
          Control_YEf = error_LON*Kp_gps + error_rate_LON*Kd_gps + GPSI_LON;
          Control_XEf = constrain(Control_XEf,-800.0f,800.0f);//PWM 1000 - 1900
          Control_YEf = constrain(Control_YEf,-800.0f,800.0f);
          //The desired roll and pitch angles by tinnakon 
          float urolldesir = (Control_YEf*m_quad)/uAltitude;//uAltitude/2 = 1000 - 1900
          float upitchdesir = (Control_XEf*m_quad*-1.0f)/uAltitude;//*-1
          urolldesir = constrain(urolldesir,-0.7f,0.7f);//+-0.7 = +-40.1 deg
          upitchdesir = constrain(upitchdesir,-0.7f,0.7f);
          float temp_YBf = asin(urolldesir)*RAD_TO_DEG;//Control Earth Frame
          float temp_XBf = asin(upitchdesir)*RAD_TO_DEG;//Control Earth Frame
          Control_XBf = (DCM00*temp_XBf) + (DCM01*temp_YBf);//Control Body Frame use Rotation matrix
          Control_YBf = (DCM10*temp_XBf) + (DCM11*temp_YBf);//Control Body Frame use Rotation matrix
          Control_XBf = constrain(Control_XBf, -20.0f, 20.0f);//+-20 deg
          Control_YBf = constrain(Control_YBf, -20.0f, 20.0f);//+-20 deg
  }
  else{
      Control_XBf = 0.0f;
      Control_YBf = 0.0f;
      GPSI_LAT = 0.0f;
      GPSI_LON = 0.0f;
      target_LAT = x1_hat;//GPS_LAT_Hold
      target_LON = y1_hat;//GPS_LON_Hold
  }
}//end  Control_PositionHold()
/////////////////////////////////////////////////////////////////////////
void Cal_GPS(){
//GPS_LAT1 = GPS_coordLAT*1e-7;///10000000.0 1e-7 degrees / position as degrees (*10E7)
//GPS_LON1 = GPS_coordLON*1e-7;
//velocity_northff = velocity_north + (gyroY_Earth*arm_GPS);//+ (gyroY_Earth*arm_GPS) velocity GPS Rotated Frame of arm gps
//velocity_eastff = velocity_east - (gyroX_Earth*arm_GPS);//- (gyroX_Earth*arm_GPS)
//Expressing latitude and longitude as linear units Earth's average meridional radius is 6367449 m
////https://en.wikipedia.org/wiki/Geographic_coordinate_system ///mn
float latMid = DEG_TO_RAD*x1_hat;
cm_per_deg_lat = (111132.92 - 559.82*cos(2*latMid) + 1.175*cos(4*latMid))*100.0d;
cm_per_deg_lon = (111412.84*cos(latMid) - 93.5*cos(3*latMid))*100.0d;
////////////////////////////////////////////////////////////////////////////////////
//float lonMid = DEG_TO_RAD*y1_hat;
//float lat2 = DEG_TO_RAD*GPS_LAT_HOME;//target_LAT
//float lon2 = DEG_TO_RAD*GPS_LON_HOME;//target_LON
//float a=atan2(sin(lon2-lonMid)*cos(lat2), cos(latMid)*sin(lat2)-sin(latMid)*cos(lat2)*cos(lon2-lonMid));
//yaw_bearing = a*RAD_TO_DEG;
}
