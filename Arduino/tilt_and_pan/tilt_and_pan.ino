// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h> 
 
Servo myservo1;  // create servo object to control a servo 
Servo myservo2;

/** pins */
int potpin1 = 0;
int potpin2 = 1;// analog pin used to connect the potentiometer
int irReader = 5; //

/** values */
int val1;    // variable to read the value from the analog pin 
int val2;
int irVal = 0;       // stores value from Ir reader
int cm; // for the irVal to cm conversion

void setup() { 
	Serial.begin( 9600 );  // begins serial communication with the computer
	myservo1.attach(9);  // attaches the servo on pin 9 to the servo object 
	myservo2.attach(10);
	//analogReference(2); //set analog reference to internal 1.1V
  	//int value = analogRead(irReader)*1100/1024;
} 
 
void loop() {
	irVal = analogRead( irReader );
	cm = 10650.08 * pow(irVal,-0.935) - 10;

	Serial.println(cm);
	

	val1 = analogRead(potpin1);
	val2 = analogRead(potpin2);  // reads the value of the potentiometer (value between 0 and 1023) 
	/*
	Serial.println("potpin1 \t potpin2");
	Serial.print(val1);
	Serial.print(" \t");
	Serial.println(val2);
	*/
	


	val1 = map(val1, 1023, 0, 0, 179);
	val2 = map(val2, 1023, 0, 0, 179);  // scale it to use it with the servo (value between 0 and 180) 
	myservo1.write(val1);
	myservo2.write(val2);  // sets the servo position according to the scaled value 
	delay(15);                           // waits for the servo to get there 
}

