#include "BLDCDriver3PWM.h"
#define _PWM_RANGE 1000.0// 2^12 -1 = 4095
#define pwm_frequency 25000 // 25khz
#define _isset(a) ( (a) != (NOT_SET) )
    bool enable_active_high = true;

BLDCDriver3PWM::BLDCDriver3PWM(int phA, int phB, int phC, int en1, int en2, int en3){
  // Pin initialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;

  // enable_pin pin
  enable_pinA = en1;
  enable_pinB = en2;
  enable_pinC = en3;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  //pwm_frequency = NOT_SET;

}

// enable motor driver
void  BLDCDriver3PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( _isset(enable_pinA) ) digitalWrite(enable_pinA, enable_active_high);
    if ( _isset(enable_pinB) ) digitalWrite(enable_pinB, enable_active_high);
    if ( _isset(enable_pinC) ) digitalWrite(enable_pinC, enable_active_high);
    // set zero to PWM
    setPwm(0,0,0);
}

// disable motor driver
void BLDCDriver3PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0, 0);
  // disable the driver - if enable_pin pin available
  if ( _isset(enable_pinA) ) digitalWrite(enable_pinA, !enable_active_high);
  if ( _isset(enable_pinB) ) digitalWrite(enable_pinB, !enable_active_high);
  if ( _isset(enable_pinC) ) digitalWrite(enable_pinC, !enable_active_high);

}

// init hardware pins
int BLDCDriver3PWM::init() {
  // PWM pins
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  if( _isset(enable_pinA)) pinMode(enable_pinA, OUTPUT);
  if( _isset(enable_pinB)) pinMode(enable_pinB, OUTPUT);
  if( _isset(enable_pinC)) pinMode(enable_pinC, OUTPUT);


  // sanity check for the voltage limit configuration
  if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  //_configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);
	PWM_Init(pwmA, 1000, pwm_frequency);
	PWM_Init(pwmB, 1000, pwm_frequency);
	PWM_Init(pwmC, 1000, pwm_frequency);
  return 0;
}



// Set voltage to the pwm pin


// Set voltage to the pwm pin
void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {

  // limit the voltage in driver
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  // hardware specific writing
  // hardware specific function - depending on driver and mcu
	pwmWrite(pwmA, dc_a*_PWM_RANGE);
	pwmWrite(pwmB, dc_b*_PWM_RANGE);
	pwmWrite(pwmC, dc_c*_PWM_RANGE);
  //_writeDutyCycle3PWM(dc_a, dc_b, dc_c, pwmA, pwmB, pwmC);
}
