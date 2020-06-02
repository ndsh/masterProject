/*******************************************
 *
 * Name.......:  Sensopoda Library
 * Description:  A powerful Library to control your HDSLR camera in various ways: IR toggle for video, adjust focus on your lens,
 *               map your focal length to the range of a stepper motor and adjust + control a supersonic sensor to measure distance.
 * Authors....:  Julian Hespenheide & Jonas Otto
 * Version....:  1.0
 * Date.......:  2013-11-20
 * Project....:  http://sensorassisted.tumblr.com
 * Contact....:  j.hespenheide@hfk-bremen.de or j.otto@hfk-bremen.de
 * License....:  This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
 *               To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/3.0/ or send a letter to
 *               Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.
 * Keywords...:  arduino, library, camera, ir, control, canon, nikon, olympus, minolta, sony, pentax, stepper, logarithmic
 * History....:  2013-11-20 V1.0 - release
 *
 ********************************************/

#if ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include "Sensopoda.h"

// ######### STANDARD LIBRARIES
// relative path  [ von Arduino/libraries/Sensopoda nach Arduino.app/[...]/Stepper.h ]
#include <../../../../../../../Booty/Applications/Arduino.app/Contents/Resources/Java/libraries/Stepper/Stepper.h>
#include <../../../../../../../Booty/Applications/Arduino.app/Contents/Resources/Java/libraries/Servo/Servo.h>

bool Phrase::_running = false;
uint8_t Phrase::_ultraPin = 0;
uint8_t Phrase::_button1 = 0;
uint8_t Phrase::_ledPin = 0;
int Phrase::_freq = 0;
int Phrase::_button1State = 0;

void wait(unsigned int time){
  unsigned long start = micros();
  while(micros()-start<=time){
  }
}

void high(unsigned int time, int freq, int pinLED) {
  int pause = (1000/freq/2)-4;
  unsigned long start = micros();
  while(micros()-start<=time) {
    digitalWrite(pinLED,HIGH);
    delayMicroseconds(pause);
    digitalWrite(pinLED,LOW);
    delayMicroseconds(pause);
  }
}

float ln2log(float mm) {
  // return log ((mm==0?0:mm)) / log (2);
  return log (mm) / log (2);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Phrase::Phrase(uint8_t button1, uint8_t button2, uint8_t ultraPin, uint8_t directionLeft, uint8_t directionRight, uint8_t servoPin, uint8_t switchPin, uint8_t stepperPin1, uint8_t stepperPin2, uint8_t stepperPin3, uint8_t stepperPin4, uint8_t ledPin, uint8_t slider1, uint8_t slider2, int stepperSpeed) {

  // allocate values to pins
  _button1 = button1; //interrupt0
  _button2 = button2;
  _ultraPin = ultraPin;  //interrupt1
  _directionLeftPin = directionLeft;
  _directionRightPin = directionRight;
  _servoPin = servoPin;
  _switchPin = switchPin;
  _stepperPin1 = stepperPin1;
  _stepperPin2 = stepperPin2;
  _stepperPin3 = stepperPin3;
  _stepperPin4 = stepperPin4;
  _ledPin = ledPin;
  _slider1 = slider1;
  _slider2 = slider2;
  _stepperSpeed = stepperSpeed;
  
  //***************************
  _running = false;
  _servoVal = 0;

  _stepperCount = 0;
  _stepperPrevious = 0;
  _servoVal = analogRead(0);

  _freq = 0;
  _currentDegree = 0.0;
  _switchState = HIGH;
  _switchReading = digitalRead(_switchPin);
  _stepperMax = 0;
  _stepperDegrees = 1.8; // stepper winkel…
  _t = 8.21;

  _directionLeftState = HIGH;
  _directionRightState = HIGH;
  
  //kalman sensitivitiy
  _kmFilteredValue = 0;
  _kmEstError = 4;
  _kmProcessNoise = 0.05;
  _kmSensorNoise = 7;
  _kmPrevFiltered = 0;

  // BUTTONS! BUTTONS! BUTTONS!
  _button1State = digitalRead(_button1);
  _button2State = digitalRead(_button2);

  // ##### ATTACH ISR routines
  attachInterrupt(_button1, ISRtoggle, CHANGE);
  attachInterrupt(_ultraPin, ISRreadDistance, CHANGE);

  // ##### INITIATE SERVO / STEPPER
  _stepper->(_stepperSpeed, _stepperPin1, _stepperPin2, _stepperPin3, _stepperPin4);
  // Stepper*( _stepper);

  // _servo = new Servo();

  // ##### SET/ATTACH PINS
  pinMode(_button1, INPUT_PULLUP);
  pinMode(_button2, INPUT_PULLUP);
  pinMode(_ultraPin, INPUT);
  pinMode(_directionLeftPin, INPUT_PULLUP);
  pinMode(_directionRightPin, INPUT_PULLUP);
  pinMode(_switchPin, INPUT_PULLUP);
  pinMode(_ledPin, OUTPUT);
  pinMode(_slider1, INPUT);
  pinMode(_slider2, INPUT);
  
  _servo->attach(_servoPin);
  _stepper->setSpeed(_stepperSpeed);

  initLensMapping();
}

Phrase::Phrase() {
  Serial.println("[Sensopoda] using lazy constructor");
  // dischitahle pinnekens
  uint8_t button1 = 2; // interrupt0
  uint8_t ultraPin = 3; // interrupt1
  uint8_t directionLeft = 4;
  uint8_t button2 = 5;
  uint8_t servoPin = 6;
  uint8_t switchPin = 7;
  uint8_t stepperPin1 = 8;
  uint8_t stepperPin2 = 9;
  uint8_t stepperPin3 = 10;
  uint8_t stepperPin4 = 11;
  uint8_t directionRight = 12;
  uint8_t ledPin = 13;

  // ahnaloche pinz
  uint8_t slider1 = 0;
  uint8_t slider2 = 1;

  // custom values for different parts and stuff and stuff and stuff
  int stepperSpeed = 100;

  Phrase(button1, button2, ultraPin, directionLeft, directionRight, servoPin, switchPin, stepperPin1, stepperPin2, stepperPin3, stepperPin4, ledPin, slider1, slider2, stepperSpeed);
}

Phrase::Phrase(bool go) {
  Serial.println("[Sensopoda] using lazy constructor 2");
  // dischitahle pinnekens
  uint8_t button1 = 2; // interrupt0
  uint8_t ultraPin = 3; // interrupt1
  uint8_t directionLeft = 4;
  uint8_t button2 = 5;
  uint8_t servoPin = 6;
  uint8_t switchPin = 7;
  uint8_t stepperPin1 = 8;
  uint8_t stepperPin2 = 9;
  uint8_t stepperPin3 = 10;
  uint8_t stepperPin4 = 11;
  uint8_t directionRight = 12;
  uint8_t ledPin = 13;

  // ahnaloche pinz
  uint8_t slider1 = 0;
  uint8_t slider2 = 1;

  // custom values for different parts and stuff and stuff and stuff
  int stepperSpeed = 100;

  Phrase(button1, button2, ultraPin, directionLeft, directionRight, servoPin, switchPin, stepperPin1, stepperPin2, stepperPin3, stepperPin4, ledPin, slider1, slider2, stepperSpeed);
}


// ######### GETTER METHOD IMPLEMENTATIONS
void Phrase::getStepper() {
  // empty
}

// ######### SETTER METHOD IMPLEMENTATIONS
/**
  set the size for the sample rate of the ultrasonic
  @param val size of samples to collect for smoothening operation of data
*/
void Phrase::setPNoise(int f) {
  if(f == 0) _kmProcessNoise -= 0.1;
  else _kmProcessNoise += 0.1;
}
void Phrase::setSNoise(int f) {
  if(f == 0) _kmSensorNoise -= 0.1;
  else _kmSensorNoise += 0.1;
}
/**
  wrapper method for setting the stepper to the correct step on the lens
*/
void Phrase::setStepper() {
  _stepperVal = analogRead(0);
  _stepper->step(_stepperVal - _stepperPrevious);
  _stepperPrevious = _stepperVal;
}
void Phrase::moveServo() {
  _servoVal = analogRead(_slider1);          // reads the value of the potentiometer (value between 0 and 1023) 
  _servoVal = map(_servoVal, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  _servo->write(_servoVal);
}

// ######### ISR(s)
void Phrase::ISRreadDistance() {
  // wow. such code. many intelligent. much complex.
  // source: http://arduino.cc/de/Reference/AttachInterrupt
}

void Phrase::ISRtoggle() {
  _button1State = digitalRead(_button1);
  if (_button1State == LOW) {
    if(_running) _running = false;
    else _running = true;
  }
  if(_running) toggleVideo();
  
}

// ######### NORMAL ROUTINE

void Phrase::update() {
  // call read functions and time them
  // get data from sensor
  // convert from ln2log -> get degrees
  
  
}
void Phrase::run() {
  // run everything. make it cool!
  // set degrees according on the stepper
  moveServo();
  if(_running) {
    _ultraPulse = pulseIn(_ultraPin, HIGH);
    float v;
    v = filtered(_kmPrevFiltered, _ultraPulse, _kmProcessNoise, _kmSensorNoise);
    v = ln2log(v);
    v = deg2step(v);
    go2step(v);
  }
}

void Phrase::initLensMapping() {
  // 1,8° stepper; 360°/1,8° = 200 steps
  // auflösung von 4:1; 1,8°/4 = .45
  // 200/.45 = 444

  // return 0 = middle Position
  // return 1 = left Position
  // return 2 = right Position
  int direction = directionSwitch();
  while(_switchState == HIGH) {
    _switchReading = digitalRead(_switchPin);
    if(_switchReading == LOW) _switchState = LOW;
    else {
      if(direction == 2) {
        _stepper->step(1);
        _stepperMax += _stepperDegrees;
      } else if(direction == 1) {
        _stepper->step(-1);
        _stepperMax -= _stepperDegrees;
      }
      
    }
  }
  _stepper->step(_stepperMax*-1);
  _stepperMax = abs(_stepperMax);
}

float Phrase::deg2step(float deg) {
  // return map(deg, 0, step2deg(_stepperMax), 0, 360);
  if(_t < deg) _t = deg;
  return mapfloat(deg, _t, 12.21, 0, 600);
  // return map(deg, 0, ln2log(5000), 0, _stepperMax);
}

void Phrase::go2step(float goalStep) {
  float a = _currentDegree-goalStep;
  float steps = abs(a);

  if(_currentDegree>goalStep){
    // goalStep = _currentDegree-goalStep;
    _stepper->step(-steps);
      Serial.print("-");
  } else if(_currentDegree<goalStep){
    // goalStep = _currentDegree+goalStep;
    _stepper->step(steps);
  }
  Serial.print(steps);
  Serial.print(" steps und currentdegree = ");
  _currentDegree = goalStep;
  
  Serial.println(_currentDegree);
  // _stepper.step(_currentDegree);

// fehlt return

}

// funktion zur berechnung des gefilterten Wertes
float Phrase::filtered(float prev,float currentValue,float q,float r) {
  /*
  x filtered Value
  p estimated error
  q process noise
  k kalman gain?
  r sensor noise
  */
  _kmEstError = _kmEstError + q; 
  float k = _kmEstError/(_kmEstError + r);
  float x=prev+k*(currentValue-prev);
  _kmEstError = (1-k) * _kmEstError;
  _kmPrevFiltered = x;

  return x;
}

int Phrase::directionSwitch() {
    // return 0 = middle Position
    // return 1 = left Position
    // return 2 = right Position
    _directionLeftState = digitalRead(_directionLeftPin);
    _directionRightState = digitalRead(_directionRightPin);
    if(_directionLeftState == HIGH && _directionRightState == LOW){
      return 1;
    } else if(_directionLeftState == LOW && _directionRightState == HIGH){
      return 2;
    } else {
      return 0;
    }
}

void Phrase::toggleVideo() {
  /* CANON */
  _freq = 33;
  for(int i=0; i<16; i++) { 
    digitalWrite(_ledPin, HIGH);
    delayMicroseconds(11);
    digitalWrite(_ledPin, LOW);
    delayMicroseconds(11);
  } 
  delayMicroseconds(5360); 
  for(int i=0; i<16; i++) { 
    digitalWrite(_ledPin, HIGH);
    delayMicroseconds(11);
    digitalWrite(_ledPin, LOW);
    delayMicroseconds(11);
  }

  /* NIKON */
  _freq = 40;
  high(2000,_freq,_ledPin);
  wait(27830);
  high(390,_freq,_ledPin);
  wait(1580);
  high(410,_freq,_ledPin);
  wait(3580);
  high(400,_freq,_ledPin);

  // Toggle Sony
  _freq = 40;
  bool _seqToggle[] = {0,0,0,1,0,0,1,0,1,0,1,1,1,0,0,0,1,1,1,1};
  for (int j=0;j<3;j++) {
    high(2320,_freq,_ledPin);
    wait(650);
    for (int i=0;i<sizeof(_seqToggle);i++){
      if (_seqToggle[i]==0){
        high(575,_freq,_ledPin);
        wait(650);
      }
      else{
        high(1175,_freq,_ledPin);
        wait(650);
      }
    }
    wait(10000);
  }

  // Toggle Olympus
  _freq = 40;
  bool _seq[] = {0,1,1,0,0,0,0,1,1,1,0,1,1,1,0,0,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1};
  high(8972,_freq,_ledPin);
  wait(4384);
  high(624,_freq,_ledPin);
  for (int i=0;i<sizeof(_seq);i++){
    if (_seq[i]==0){
      wait(488);
      high(600,_freq,_ledPin);
    }
    else{
      wait(1600);
      high(600,_freq,_ledPin);
    }
  };

  // Toggle Minolta
  _freq = 38;
  bool _seq2[] = {
    0,0,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1};
  high(3750,_freq,_ledPin);
  wait(1890);
  for (int i=0;i<sizeof(_seq2);i++){
    if (_seq2[i]==0){
      high(456,_freq,_ledPin);
      wait(487);
    }
    else{
      high(456,_freq,_ledPin);
      wait(1430);
    }
  };

  // Toggle Pentax
  _freq = 38;
  high(13000,_freq,_ledPin);
  wait(3000);
  for (int i=0;i<7;i++){
    high(1000,_freq,_ledPin);
    wait(1000);
  };

}