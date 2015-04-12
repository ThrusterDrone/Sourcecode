int gyroPin = 0;               //Gyro is connected to analog pin 0
float gyroVoltage = 5;         //Gyro is running at 5V
float gyroZeroVoltage = 2.5;   //Gyro is zeroed at 2.5V
float gyroSensitivity = .007;  //Our example gyro is 7mV/deg/sec
float rotationThreshold = 1;   //Minimum deg/sec to keep track of - helps with gyro drifting
float currentAngle = 0;          //Keep track of our current angle

float temp;
int tempPin = 1;
int alarmtemp = 10 ; 

int valve1 = 2 ; 
int valve2 = 3 ; 
int valve3 = 4 ;
int valve4 = 5 ; 
int valve5 = 22 ;
int valve6 = 7 ; 
int valve7 = 8 ; 
int valve8 = 9 ; 

int echipin1 = A0 ; 
int echipin2 = A1 ;
int echipin3 = A2 ;
int echipin4 = A3 ;
int echipin5 = A4 ;
int echipin6 = A5 ;

int measurePin = 6;
int ledPower = 12; 
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

#include <NewPing.h>

#define SONAR_NUM     15 // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(41, 42, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(43, 44, MAX_DISTANCE),
  NewPing(45, 20, MAX_DISTANCE),
  NewPing(21, 22, MAX_DISTANCE),
  NewPing(23, 24, MAX_DISTANCE),
  NewPing(25, 26, MAX_DISTANCE),
  NewPing(27, 28, MAX_DISTANCE),
  NewPing(29, 30, MAX_DISTANCE),
  NewPing(31, 32, MAX_DISTANCE),
  NewPing(34, 33, MAX_DISTANCE),
  NewPing(35, 36, MAX_DISTANCE),
  NewPing(37, 38, MAX_DISTANCE),
  NewPing(39, 40, MAX_DISTANCE),
  NewPing(50, 51, MAX_DISTANCE),
  NewPing(52, 53, MAX_DISTANCE)
};


void setup() {
  Serial.begin (9600);
 pinMode( valve1, OUTPUT) ;
 pinMode( valve2, OUTPUT) ;
 pinMode( valve3, OUTPUT) ;
 pinMode( valve4, OUTPUT) ;
 pinMode( valve5, OUTPUT) ;
 pinMode( valve6, OUTPUT) ;
 pinMode( valve7, OUTPUT) ;
 pinMode( valve8, OUTPUT) ;
 
 pinMode(echoPin1, INPUT) ;
 pinMode(echoPin2, INPUT) ;
 pinMode(echoPin3, INPUT) ;
 pinMode(echoPin4, INPUT) ;
 pinMode(echoPin5, INPUT) ;
 pinMode(echoPin6, INPUT) ; 
 
 pinMode(alarmtemp, OUTPUT);
 
 pinMode(ledPower,OUTPUT);
 
}

pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}


void loop() {
  //This line converts the 0-1023 signal to 0-5V
  float gyroRate = (analogRead(gyroPin) * gyroVoltage) / 1023;

  //This line finds the voltage offset from sitting still
  gyroRate -= gyroZeroVoltage;

  //This line divides the voltage we found by the gyro's sensitivity
  gyroRate /= gyroSensitivity;

  //Ignore the gyro if our angular velocity does not meet our threshold
  if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) {
    //This line divides the value by 100 since we are running in a 10ms loop (1000ms/10ms)
    gyroRate /= 100;
    currentAngle += gyroRate;
  }

  //Keep our angle between 0-359 degrees
  if (currentAngle < 0)
    currentAngle += 360;
  else if (currentAngle > 359)
    currentAngle -= 360;

  //DEBUG
  Serial.println(currentAngle);

  delay(10);
  
  temp = analogRead(tempPin);
  temp = temp * 0.48828125;
  if( temp >=23 ) ;
 { digitalWrite ( alarmtemp, HIGH ) ; 
  Serial.print ("warning temp =" ) ;
  Serial.print (temp);}
  else {
  Serial.print("TEMPRATURE = ");
  Serial.print(temp);
  Serial.print("*C");
  Serial.println();
  delay(1000);
  }
   digitalWrite(ledPower,LOW); // power on the LED
  delayMicroseconds(samplingTime);
 
  voMeasured = analogRead(measurePin); 
 
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH); 
  delayMicroseconds(sleepTime);
 
  calcVoltage = voMeasured * (3.3 / 1024);

  dustDensity = 0.17 * calcVoltage - 0.1;
 
  Serial.print("Raw Signal Value (0-1023): ");
  Serial.print(voMeasured);
 
  Serial.print(" - Voltage: ");
  Serial.print(calcVoltage);
 
  Serial.print(" - Dust Density: ");
  Serial.println(dustDensity);
 
  delay(1000);

}
for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  // The rest of your code would go here.
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
}
}

