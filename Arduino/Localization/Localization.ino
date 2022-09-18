
#include <TimerOne.h>
#include <MPU6050.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

TimerOne ISR;

const byte MOTOR1 = 3;  


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
// Pitch, Roll and Yaw values

float yaw = 0;
float t1,t2;
float omega ; 

float r= 0.025 ;

float X_car = 0 ;
float Y_car = 0 ;
float velocity ; // rpm 
unsigned int counter = 0; //  pulses counter
float diskslots = 20; 
float Vx = 0  ; 
float Vy = 0 ; 
float real_velocity ; 
float time_constant ;

void setup() {
      Serial.begin(115200);
      init_IMU();
}

void loop()
{
    t1 = millis() ;
    Vector norm = mpu.readNormalizeGyro();

    Vx = real_velocity*cos(yaw*180 / 3.14) ; 
    Vy = real_velocity*sin(yaw*180 / 3.14) ; 

    // Calculate Yaw

    yaw = yaw + norm.ZAxis * timeStep;

    time_constant= timeStep ; 
    X_car = X_car + Vx*time_constant  ;
    Y_car= Y_car + Vy*time_constant ;
    
    communication();
    
    t2 = millis();
//    timeStep = t2 - t1;
}

void init_IMU()
{
 
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
    // Calibrate gyroscope. The calibration must be at rest.
    mpu.calibrateGyro();

    // Set threshold sensivty. Default 3.
    mpu.setThreshold(3);
  
    // Encoder
    Timer1.initialize(1000000); 
    attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count, RISING);  
    Timer1.attachInterrupt( ISR_timerone ); 
}

void ISR_timerone()
{   
    Timer1.detachInterrupt();  
    velocity = (counter / diskslots) * 60.00; 
    omega = velocity*2*3.14/60 ; 
    real_velocity = omega*r ;  

    counter = 0;  
    Timer1.attachInterrupt( ISR_timerone );  // enVble the timer
}

void ISR_count()  
{
    counter++;
}

void communication()
{
//    Serial.print( "      Real V    " ) ;
    Serial.print( real_velocity ) ; 
    Serial.print( "," ) ;
//    Serial.print( "       X     " ) ;
    Serial.print( X_car ) ; 
    Serial.print( "," ) ;
//    Serial.print("     Y       ") ;    
    Serial.print( Y_car) ;
    Serial.print( "," ) ;
//    Serial.print( "   theta     " ) ; 
    Serial.print( yaw) ;
    Serial.println("");
}
