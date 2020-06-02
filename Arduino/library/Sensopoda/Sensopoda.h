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

/**

PARTS:
    Stepper (4 pins)
    Servo (1 pin)
    Ultrasonic (1 pin)
    Switch (1 pin)


INPUTS:
    Ultrasonic (amount)



OUTPUTS:
    Stepper
    Servo
    LED(s) (video: an/aus)
*/


#ifndef Sensopoda_h
#define Sensopoda_h

#if ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

/* forwarding der libraries funktioniert nicht */
class Stepper;
class Servo;

class Phrase {
    private:
        static uint8_t _button1;
        uint8_t _button2;
        static uint8_t _ultraPin;
        uint8_t _directionLeftPin;
        uint8_t _directionRightPin;
        uint8_t _servoPin;
        
        uint8_t _switchPin;
        uint8_t _stepperPin1;
        uint8_t _stepperPin2;
        uint8_t _stepperPin3;
        uint8_t _stepperPin4;
        static uint8_t _ledPin;
        uint8_t _slider1;
        uint8_t _slider2;

        double _stepperCount;
        double _stepperMax;
        double _stepperDegrees;
        
        int _servoVal;
        int _stepperSpeed;
        int _stepperPrevious;
        int _stepperVal;
        static int _freq;
        int _currentDegree;
        
        int _ultraPulse;
        Stepper* _stepper;
        Servo* _servo;
        
        
        int _switchState;
        int _switchReading;
        uint8_t _buttonPin;
        float _t;
        uint8_t _directionLeftState;
        
        uint8_t _directionRightState;

        static int _button1State;
        int _button2State;

        static bool _running;

        float _kmFilteredValue;
        float _kmEstError;
        float _kmProcessNoise;
        float _kmSensorNoise;
        float _kmPrevFiltered;
        
    public:   
        Phrase(uint8_t button1, uint8_t button2, uint8_t ultraPin, uint8_t directionLeft, uint8_t directionRight, uint8_t servoPin, uint8_t switchPin, uint8_t stepperPin1, uint8_t stepperPin2, uint8_t stepperPin3, uint8_t stepperPin4, uint8_t ledPin, uint8_t slider1, uint8_t slider2, int stepperSpeed);
        Phrase(); // default for lazy people
        Phrase(bool go); // default for lazy people
        void update();
        void run();
        static void toggleVideo();
        void setStepper();
        void getStepper();
        void moveServo();
        static void ISRtoggle();
        static void ISRreadDistance();
        void initLensMapping();
        float deg2step(float deg);
        void go2step(float step);
        float filtered(float prev, float currentValue, float q, float r);
        int directionSwitch();

        // getters

        // setters
        void setPNoise(int f);
        void setSNoise(int f);
};

#endif