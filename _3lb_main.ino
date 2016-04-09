#include <math.h>
#include <RedBot.h>
#include <SabertoothSimplified.h>
#include <Servo.h>

#define LIGHT_IN_PIN1 A2
#define LIGHT_IN_PIN2 A3
#define WEAPON_PIN 9
#define AUTO_SWITCH 2

#define AUTO_ON 2028
#define AUTO_OFF 1024

#define HALF_POWER 63
#define QUARTER_POWER 31

//sensor calibration variables
double avgSensorValBlack1 = 0;
double stdDevBlack1 = 0;
double avgSensorValBlack2 = 0;
double stdDevBlack2 = 0;
int numTrials = 1000;

volatile bool autoMode = false;       //start in manual mode
volatile long startTime = 0;          //time when auto mode was toggled (in ms)
const long autoDuration = 180000;     //3 minute duration for autonomous mode

volatile unsigned long timer_start;    //start of switch pulse time
volatile int pulse_time;               //length of switch pulse time

//random number generated when deciding how to react to white
long randNumber;

//light sensors
RedBotSensor sensor1 = RedBotSensor(LIGHT_IN_PIN1);
RedBotSensor sensor2 = RedBotSensor(LIGHT_IN_PIN2);

//weapon servo
Servo weaponServo;
int pos;

//motor control
SabertoothSimplified ST;

void setup() {
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  
  //setup auto swtich pin
  attachInterrupt(digitalPinToInterrupt(AUTO_SWITCH), checkAutonomous, CHANGE);
  
  //Serial
  SabertoothTXPinSerial.begin(9600);
  Serial.begin(9600);
  
  //calibrate the sensors
  calibrate();
  
  //generate the seed for the autonomous reactions
  randomSeed(sensor1.read());
  
  //attach the weapon
  weaponServo.attach(WEAPON_PIN);
  
  //start off stopped
  ST.motor(1,0);
  ST.motor(2,0);
}

void loop() {
  if (autoMode == true) {
    autonomousMode();
  }
  else {
    manualMode();
  }
}

//finds the average value for BLACK
//NOTE: must be run with robot on BLACK surface prior to main logic
void calibrate() {
  //Setup robot's reference variables to environment
  for(int i = 0; i < numTrials; i++) {
      avgSensorValBlack1 += sensor1.read();
      avgSensorValBlack2 += sensor2.read();
  }
  avgSensorValBlack1 /= numTrials;
  avgSensorValBlack2 /= numTrials;
  
  for(int i = 0; i < numTrials; i++) {
      double val1 = avgSensorValBlack1 - sensor1.read();
      double val2 = avgSensorValBlack2 - sensor2.read();
      stdDevBlack1 += val1 * val1;
      stdDevBlack2 += val2 * val2;
  }
  stdDevBlack1 = sqrt(stdDevBlack1);
  stdDevBlack2 = sqrt(stdDevBlack2);
}

//tusns on autonomous mode when the switch is flipped, and turns it off when either
//the timer runs out or the switch is flipped back
void checkAutonomous() {
  //if the pin has gone HIGH, record the microseconds since the Arduino started up 
  if (digitalRead(AUTO_SWITCH) == HIGH) { 
      timer_start = micros();
  } 
  //otherwise, the pin has gone LOW 
  else { 
    //only worry about this if the timer has actually started
    if(timer_start != 0) { 
      //record the pulse time
      pulse_time = ((volatile int)micros() - timer_start);
      
      Serial.println(pulse_time);
      
      //tuirn on auto mode
      if (pulse_time == AUTO_ON) {
        //stop auto mode if the timer has run out
        if (autoMode == true && millis() - startTime > autoDuration) {
          autoMode = false;
        }
        //if the switch was just flipped, turn on autonomous mode
        else if (autoMode == false && startTime == 0) {
          autoMode = true;
          startTime = millis();
        }
      }
      //turn off auto mode when switch flipped down
      else if (pulse_time == AUTO_OFF) {
        autoMode = false;
        startTime = 0;
      }
      
      //restart the timer
      timer_start = 0;
    }
  } 
}

//run based off of the manual controls of the rc controller
void manualMode() {
  //TODO, maybe
  Serial.println("In manual mode");
  
  //temp: stop the robot
  ST.motor(1,0);
  ST.motor(2,0);
}

//run autonomously
void autonomousMode() {
  //Regular sensor values:
  //White: LOW
  //Black: HIGH
  
  //Read diode sensor values
  int sensor1Level = sensor1.read();
  int sensor2Level = sensor2.read();
  
  Serial.print("Sensor 1 level: ");
  Serial.println(sensor1Level);
  Serial.print("Sensor 2 level: ");
  Serial.println(sensor2Level);
  
  //Stop if white on either sensor
  if((sensor1Level > avgSensorValBlack1 + stdDevBlack1 || sensor1Level < avgSensorValBlack1 - stdDevBlack1) ||
     (sensor2Level > avgSensorValBlack2 + stdDevBlack2 || sensor2Level < avgSensorValBlack2 - stdDevBlack2)) 
  {
    stopDrive();
  }
  //Drive if black 
  else 
  {
    runDriveForward(HALF_POWER);
  }
  
  //delay(50);
  
  useWeapon();
}

//move forward with the specified power
void runDriveForward(int pwr) {
  ST.motor(1, pwr);
  ST.motor(2, pwr);
  delay(20);
}

//stop the robot, reverse, and turn in a random direction 
//used in avoiding going out of bounds
void stopDrive() {
  //wait slightly before stopping to not put strain on the motors
  delay(50);
  
  //stop
  ST.motor(1, 0);
  ST.motor(2, 0);
  delay(1000);
  
  //reverse away from the edge at 25% power
  ST.motor(1, -QUARTER_POWER);
  ST.motor(2, -QUARTER_POWER);
  delay(1000);
  
  //stop breifly before turning
  ST.motor(1, 0);
  ST.motor(2, 0);
  delay(1000);
  
  //randomly decide to pivot left or right to avoid the white
  //0-49 = turn left, 50-99 = turn right
  randNumber = random(100);
  if (randNumber < 50) {
    //pivot turn left
    ST.motor(1, -HALF_POWER);
    ST.motor(2, HALF_POWER);
  }
  else {
    //pivot turn right
    ST.motor(1, HALF_POWER);
    ST.motor(2, -HALF_POWER);
  }
  delay(1000);
}

void useWeapon() {
  //with continuous rotation servo, servo.write(x) sets the speed of the servo
  //where 45 is full-speed ccw, 0 and 90 are stopped, and 135 is full speed cw
  
  //spin full speed ccw
  pos = 45;
  weaponServo.write(pos);
  delay(15);
}
