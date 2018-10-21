/*
support: 
• ESP8266
• GY86 ,MPU6050 6 axis gyro/accel with Motion Processing Unit
• Bi-Copter Frame
*/
//PID-------------Rate
float Kp_rateRoll = 47.2;//45.2 55 65 95 135 165 185.2 225 60.00 80
float Ki_rateRoll = 0.35;//0.25
float Kd_rateRoll = 4.31;//4.10 10.0 5.00

float Kp_ratePitch = 27.2;//24.2 29 15 25 45.2 185.2 225
float Ki_ratePitch = 8.35;//9.3 15 5
float Kd_ratePitch = 0.62;//0.82 0.4 1.2 8.10

float Kp_rateYaw = 42.1;//45 65 160 260 350
float Ki_rateYaw = 35.51;//30 28 18 2.5
float Kd_rateYaw = 0.0025;

//PID--------------Stable
float Kp_levelRoll= 9.51;//8.51 6.0 
float Ki_levelRoll= 0.85;//0.0
float Kd_levelRoll= 0.02;//0.0

float Kp_levelPitch= 16.51;//15.51 13 8.5 11 9 6.5
float Ki_levelPitch= 0.0;//0.8
float Kd_levelPitch= 0.0;//0.2

//stat feedback--------------Altitude
float Kp_altitude = 180.2f;//155.2 225.2 265.2  175.0  165.0
float Ki_altitude = 11.45f;//2.1 12.25 52.13 2.13 0.018 2.5,0.0
float Kd_altitude = 235.2f;//210.2 185.2 195.2 185.2 250 280.5 315.5 120
float Kd2_altitude = 33.4f;//23.4 35.25 18.25 22.25 42.5 12.2 1.25
float Ka_altitude = 0.11f;//8.5 18.5 26 32.5 38.5 41.5 35 25 - 45
//PID GPS////////////////////////////////////////////
float Kp_gps = 0.135f;//0.1405 0.245 0.15 0.101 2.101 5.101
float Ki_gps = 0.122f;//0.122 0.68 2.68 0.25 0.085 0.15
float Kd_gps = 1.531f;//1.850 2.85 3.35 4.35 1.05 1.9 4.3 0.35 1.35 3.35
float Kp_speed = 0.437f;//0.247 0.43 0.37 0.27 0.15 0.35 0.095 min 0.15

int servo_trim_L = 1460;//1568 1330 1400
int servo_trim_R = 1560;//1477 1780 1700

//GPS //สตาร์ท//////////////////////////////////////
double GPS_LAT_HOME = 13.8681811d;//13.8681811d ,13.868180,100.496297 13868180
double GPS_LON_HOME = 100.4962971d;//100.4962971d
//ลงจอด
double waypoint1_LAT = 13.8681811d;//13.868544, 100.496433
double waypoint1_LON = 100.4962971d;//B,
/////////////////////////////
double target_LAT = GPS_LAT_HOME;//13.8681811d ,13.868180 100.496297
double target_LON = GPS_LON_HOME;//100.4962971d
// Automatic take-off and landing 
float h_control = 3.5f;  //2.7 0.6 0.9 meter
float Altitude_Max = 20.0f;//max 20 meter
//Parameter system bi-rotor/////////////////////////////////////////
float m_quad = 0.71f;     //kg
float L_quad = 0.18f;   //m quad+=0.25   I = (1/3)mL2 = 0.02291
float c_quad = 0.00364f; //N.s/m
float c_XYquad = 0.0000364f; //N.s/cm
//float k_XYquad = 4.364;//N/cm ,,8.364 10.364
float arm_GPS = 0.1f; //17 cm
float Fight_time = 480.1f; //s = 8 min
/////////////////////////////////////////////////////////////////////
///////////////Mode///////////////////////////
#define Mannual 1018
#define AltHold 1295
#define RTH 1707
#define PositionHold 1545  //Loiter mode
#define Auto 1965
int Mode = 0;
//Mode 0 = Stabilize
//Mode 1 = Altitude Hold
//Mode 2 = Position Hold ,Loiter
//Mode 3 = Automatic  Takeoff ,Landing
//magnetometer calibration constants; use the Calibrate example from
// the Pololu library to find the right values for your board
int M_X_MIN = -320;    //-654  -693   -688
int M_X_MAX = 210;     //185   209    170
int M_Y_MIN = -210;    //-319  -311   -310
int M_Y_MAX = 300;     //513   563    546
int M_Z_MIN = -540;    //-363  -374   -377
int M_Z_MAX = 35;     //386   429    502
////////////////////////////////////////////////////////////////////
#define TASK_100HZ 1  //3
#define TASK_50HZ 2  //5
#define TASK_20HZ 5
#define TASK_10HZ 10 //25
#define TASK_5HZ 20
#define TASK_1HZ 100 //100 250

// sub loop time variable
unsigned long oneHZpreviousTime = 0;
unsigned long tenHZpreviousTime = 0;
unsigned long lowPriorityTenHZpreviousTime = 0;
unsigned long lowPriorityTenHZpreviousTime2 = 0;
unsigned long fiftyHZpreviousTime = 0;
unsigned long hundredHZpreviousTime = 0;
//unsigned long frameCounter = 0; // main loop executive frame counter

// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;
uint8_t frameCounter = 0;
uint32_t itterations = 0;
float G_Dt = 0.01; 

byte mode = 1;

//#define PI 3.141592653589793f
#define D_LPF_HZ 20
#define LOOP_S 0.01f //assuming we are running at 200Hz

static float d_lpf = LOOP_S / (LOOP_S + 1/(2*PI*D_LPF_HZ));

//roll angle
float r_Kp=5.0f;
float r_Ki=0.0f;
float r_Kd=0.0f;
float r_max=0.0f;
float r_imax=0.0f;
float r_value = 0.0f;
float r_lastInput = 0.0f;
float r_lastDInput = 0.0f;
float r_KiTerm = 0.0f;
float r_dInput = 0.0f;

//roll rate
float rate_r_Kp=1.0f;
float rate_r_Ki=0.55;
float rate_r_Kd=0.0f;
float rate_r_max=0.0f;
float rate_r_imax=0.0f;
float rate_r_value = 0.0f;
float rate_r_lastInput = 0.0f;
float rate_r_lastDInput = 0.0f;
float rate_r_KiTerm = 0.0f;
float rate_r_dInput = 0.0f;

//pitch level
float p_Kp=5.0f;
float p_Ki=0.0f;
float p_Kd=0.0f;
float p_max=0.0f;
float p_imax=0.0f;
float p_value = 0.0f;
float p_lastInput = 0.0f;
float p_lastDInput = 0.0f;
float p_KiTerm = 0.0f;
float p_dInput = 0.0f;

//pitch rate
float rate_p_Kp=1.0f;
float rate_p_Ki=0.55f;
float rate_p_Kd=0.0f;
float rate_p_max=0.0f;
float rate_p_imax=0.0f;
float rate_p_value = 0.0f;
float rate_p_lastInput = 0.0f;
float rate_p_lastDInput = 0.0f;
float rate_p_KiTerm = 0.0f;
float rate_p_dInput = 0.0f;

//yaw level
float y_Kp=0.0f;
float y_Ki=0.0f;
float y_Kd=0.0f;
float y_max=0.0f;
float y_imax=0.0f;
float y_value = 0.0f;
float y_lastInput = 0.0f;
float y_lastDInput = 0.0f;
float y_KiTerm = 0.0f;
float y_dInput = 0.0f;

//yaw rate
float rate_y_Kp=3.0f;
float rate_y_Ki=0.0f;
float rate_y_Kd=.0f;
float rate_y_max=0.0f;
float rate_y_imax=0.0f;
float rate_y_value = 0.0f;
float rate_y_lastInput = 0.0f;
float rate_y_lastDInput = 0.0f;
float rate_y_KiTerm = 0.0f;
float rate_y_dInput = 0.0f;

//float r_input=0.0;
//float p_input=0.0;
//float y_input=0.0;// -(ahrs_r*180/PI);
			
float loop_s = 0.01f;    

float roll_I_rate;
float roll_D_rate;
float err_roll_rate;
float err_roll_ant_rate;

float pitch_I_rate;
float pitch_D_rate;
float err_pitch_rate;
float err_pitch_ant_rate;

float yaw_I_rate;
float yaw_D_rate;
float err_yaw_rate=0;
float err_yaw_ant_rate=0;


float roll_I_level;
float roll_D_level;
float err_roll_level=0;
float err_roll_ant_level=0;

float pitch_I_level;
float pitch_D_level;
float err_pitch_level=0;
float err_pitch_ant_level=0;

float altitude_I;
float altitude_D;
float err_altitude=0;
float err_altitude_ant=0;

float last_input_roll=0;
float last_input_pitch=0;

float control_roll_angle=0.0;          
float control_pitch_angle=0.0;
float control_yaw_angle=0.0;


float control_roll=0.0;          
float control_pitch=0.0;
float control_yaw=0.0;

float holdAltitude=0.0;
float altitude=0.0;
int thr=0;


#define tar 0.001
byte armed;

float velocityCompFilter1 = 1.0 / (1.0 + 0.3);
float velocityCompFilter2 = 1 - velocityCompFilter1;

boolean runtimaZBiasInitialized = false;  
float zVelocity = 0.0;
float estimatedZVelocity = 0.0;
float runtimeZBias = 0.0; 
float zDampeningThrottleCorrection = 0.0;


float filteredAccel[3] = {0.0,0.0,0.0};
int motor_FRONTL;
int motor_FRONTR;
int servo_REARL;
int servo_REARR;
//int LED_BUILTIN = D0;
//direction cosine matrix (DCM)   Rotated Frame to Stationary Frame ZYX
float DCM00 = 1.0f;
float DCM01 = 0.0f;
float DCM02 = 0.0f;
float DCM10 = 0.0f;
float DCM11 = 1.0f;
float DCM12 = 0.0f;
float DCM20 = 0.0f;
float DCM21 = 0.0f;
float DCM22 = 1.0f;

static int32_t GPS_degree[2];
int32_t _velocity_north;
int32_t _velocity_east;
byte buf1[16];//data 16 byte lat 4 byte , lon 4 byte, _velocity_north 4 byte ,,_velocity_east 4 byte
double cm_per_deg_lat = 11113295.451d;
double cm_per_deg_lon = 11113195.451d;

//kalman//////////////////////////////////////
double x1_xt = 0.0d;
float x2_vt = 0.0f;
float x3_at = 0.0f;
double y1_xt = 0.0d;
float y2_vt = 0.0f;
float y3_at = 0.0f;
double vx1_hat=0.0d;
float vx2_hat=0.0f;
float vx3_hat=0.0f;
double vy1_hat=0.0d;
float vy2_hat=0.0f;
float vy3_hat=0.0f;
double x1_hat = GPS_LAT_HOME;
double y1_hat = GPS_LON_HOME;

float z1_hat = 0.0f;
float z2_hat = 0.0f;
float z3_hat = 0.0f;
float z3_acc = 0.0f;
float z1_hat2 = 0.0f;
float z2_hat2 = 0.0f;
float u_z = 0.0f;
float u_zold = 0.0f;
float u_zdot = 0.0f;
float u_Ydot = 0.0f;
float u_Xdot = 0.0f;
float accrX_Earth = 0.0f;
float accrY_Earth = 0.0f;
float accrZ_Earth = 0.0f;
float accrX_Earthf = 0.0f;
float accrY_Earthf = 0.0f;
float accrZ_Earthf = 0.0f;
float AccXf,AccYf,AccZf;
float acc_offsetZ2 = 9.80665f;
float Velocity_THR = 0.0f;
float Altitude_Hold = 0.0f;
float uthrottle=0.0f;
float I_throttle = 1050.0f;
float uAltitude = 1000.0f;

//Automatic take-off and landing
float err_hz = 0.0f;
int time_auto = 0;
float h_counter = 0.1f;//0.08
float h_counter_old = 0.1f;
float Vz_Hold = 0.0f;
float hz_I = 0.0f;
float hz_D_rate = 0.0f;
float error_Vz_old = 0.0f;
uint8_t takeoff = 0;
uint8_t endAuto = 0;
uint8_t Status_waypoint = 0;
float error_LAT = 0.0f;
float error_LON = 0.0f;  
float targetRE_speedLAT = 0.0f;//vX Earth Frame
float targetRE_speedLON = 0.0f;//vY Earth Frame
float GPSI_LAT = 0.0f;
float GPSI_LON = 0.0f; 
float Control_XEf = 0.0f;
float Control_YEf = 0.0f;
float Control_XBf = 0.0f;
float Control_YBf = 0.0f;

uint8_t timeLanding = 0;
uint8_t timeOff = 0;

//Baro
MS561101BA baro = MS561101BA();
float temperature = 25.20f;
float presser = 1003.52f;
float sea_press=1013.25f;
float Altitude_baro=0.0f;
float Altitude_barof=0.0f;
float Altitude_baroEarth=0.0f;
float Vz_Baro_ult=0.0f;
float Vz_Baro_ultold=0.0f;
