/*
support: 
• ESP8266
• GY86 ,MPU6050 6 axis gyro/accel with Motion Processing Unit
• Bi-Copter Frame
*/
uint8_t  interruptPin = D3;
uint8_t chan1 = 0;
volatile uint32_t rcValue[8] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500};
int CH_THR = 1000;
int CH_AIL = 1500;
int CH_ELE = 1500;
int CH_RUD = 1500;
int AUX_1 = 1000;
int AUX_2 = 1000;
int AUX_3 = 1000;
int AUX_4 = 1000;

void pwmHandler(int ch, int pin) {
  ///PPM/////////////////// Arduino Due RX PPM
  uint32_t now, diff;
  static uint32_t last = 0;
  now = micros();
  diff = now - last;
  last = now;
  if (diff > 3000) chan1 = 0;//3000 us
  else {
    if (900 < diff && diff < 2200 && chan1 < 8) { //900 - 2200 us
      rcValue[chan1] = diff;
    }
    chan1++;
  }
}
void ch1Handler() {
  pwmHandler(0, interruptPin);
}

void remotePPM_initialize()
{
pinMode(interruptPin, INPUT);//INPUT_PULLUP
attachInterrupt(interruptPin, ch1Handler, RISING);
}
void computeRC()
{
 CH_THR = rcValue[2];
 CH_AIL = rcValue[0];
 CH_ELE = rcValue[1];
 CH_RUD = rcValue[3];
 AUX_1 = rcValue[4];
 AUX_2 = rcValue[5];
 AUX_3 = rcValue[6];
 AUX_4 = rcValue[7];
}

