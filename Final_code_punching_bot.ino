#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <math.h>
//#include <ArduinoEigen.h>
//#include <ArduinoEigenDense.h>
//#include <ArduinoEigenSparse.h>

//--------------------------------------------------------------------------
// Code to test encoders
// 04.11.14
// Last updated by Nandan Seshadri
//--------------------------------------------------------------------------

// Pin declares
int solenoidPin = 11;
const int encoder1PinA = 2;
const int encoder1PinB = 3;
int pwmPin1 = 9; // PWM output pin for motor 1
int dirPin1 = 8; // direction output pin for motor 1
const int encoder2PinA = 18;
const int encoder2PinB = 19;
int pwmPin2 = 6; // PWM output pin for motor 1
int dirPin2 = 7; // direction output pin for motor 1

// Variables to hold the degree value
 float degree1 = 0;     // For base motor
 float degree2 = 0;     // For end motor
 double g = 9.81;


// Jacobian matrix
//double J[2][2] = {{0.0, 0.0}, {0.0, 0.0}};

 //*************************************************************************************************
//******************************RENDERING VIRTUAL ENVIRONMENTS*************************************
//*************************************************************************************************
#define virtual_wall        // Enables virtual wall at base
#define mass_spring
#define L1 0.254
#define L2 0.2032
//*************************************************************************************************
//*************************************************************************************************
//*************************************************************************************************

// Variables to hold encoder position data
long encoder1Pos = 0;
long encoder2Pos = 0;
int encoder1Last = 0;
int encoder2Last = 0;

// Kinematics variables
double x = 0;
double y = 0;
double theta1, theta2;
//double xh1 = 0;           // position of the handle for base [m]
double xh2 = 0;           // position of the handle for end [m]
double J_transpose[2][2];  // Transpose of the Jacobian matrix
double x_prev = 0; //IC previous handle position
double x_mass = 0.01; //IC mass position
double x_massdot = 0; // IC of velocity for mass spring damper simulation
double x_massdotdot = 0; //IC of acceleration for mass spring damper simulation
double new_value = 0; //approximation of derivative of handle position
double old_value = 0; //previous velocity value
double vel = 0; //IC of handle velocity
double alpha = 0.4; //time constant for velocity filter
double x_wall = 0.005; //position of wall

// Force output variables
double force1 = 0;           // force at the handle
double Tp1[2][2];              // torque of the motor pulley
double duty1 = 0;            // duty cylce (between 0 and 255)
unsigned int output1 = 0;    // output command to the motor
double force2 = 0;           // force at the handle
double Tp2 = 0;              // torque of the motor pulley
double duty2 = 0;            // duty cylce (between 0 and 255)
unsigned int output2 = 0;    // output command to the motor

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  // Set up serial communication
  Serial.begin(115200);

  // Solenoid 
  pinMode(solenoidPin, OUTPUT);

  // Set encoder pins as inputs
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  pinMode(encoder2PinA, INPUT);
  pinMode(encoder2PinB, INPUT);

  // Enable internal pull-up resistors for encoder pins
  digitalWrite(encoder1PinA, HIGH);
  digitalWrite(encoder1PinB, HIGH);
  digitalWrite(encoder2PinA, HIGH);
  digitalWrite(encoder2PinB, HIGH);

  // Attach interrupt service routines to encoder pins
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), encoder2ISR, CHANGE);
  
  // Set PWM frequency 
//  TCCR4B = TCCR4B & B11111000 | B00000001;   // for PWM frequency D6 of 31372.55 Hz
//  TCCR2B = TCCR2B & B11111000 | B00000001;  // for PWM frequency D9 of 31372.55 Hz
  setPwmFrequency1(pwmPin1,1); 
  setPwmFrequency2(pwmPin2,1);
  
  // Output pins
  pinMode(pwmPin1, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin1, OUTPUT);  // dir pin for motor A
  pinMode(pwmPin2, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin2, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin1, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin1, LOW);  // set direction
  analogWrite(pwmPin2, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin2, LOW);  // set direction  

//  // Initialize Jacobian matrix
//  J << -L1*sin(q1)-L2*sin(q1+q2), -L2*sin(q1+q2),
//        L1*cos(q1)+L2*cos(q1+q2),  L2*cos(q1+q2);

  // Transpose Jacobian matrix
//  J_transpose = J.transpose();
}

// Interrupt service routine for encoder 1
void encoder1ISR() {
  int newEncoder1 = digitalRead(encoder1PinA);
  if ((encoder1Last == LOW) && (newEncoder1 == HIGH)) {
    if (digitalRead(encoder1PinB) == LOW) {
      encoder1Pos--;
    } else {
      encoder1Pos++;
    }
  }
  encoder1Last = newEncoder1;
}

// Interrupt service routine for encoder 2
void encoder2ISR() {
  int newEncoder2 = digitalRead(encoder2PinA);
  if ((encoder2Last == LOW) && (newEncoder2 == HIGH)) {
    if (digitalRead(encoder2PinB) == LOW) {
      encoder2Pos--;
    } else {
      encoder2Pos++;
    }
  }
  encoder2Last = newEncoder2;
}

// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{

  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // Base encoder
  // Define kinematic parameters you may need
   double degree1 = encoder1Pos*(360/64); // Motor - 64 CPR
   double theta1 = degree1/19;    // Gear ratio 19:1
//   Serial.print(theta1);
//   xh1 = L1*(degree1*3.14159265/180);       // Compute the position of the handle (in meters) based on ts (in radians)
//   Serial.print(xh1, 5);

  // End encoder
  // Define kinematic parameters you may need
   double degree2 = encoder2Pos*(180/64);   // 64 CPR
   double theta2 = degree2/10;    // Gear ratio is 10:1
//   Serial.println(theta2);
//   Serial.println(xh2, 5);

  // Forward kinematics
  // Calculate the end effector position
//  x = L1 * cos(theta1*3.14159265/180) + L2 * cos(theta1*3.14159265/180 + theta2*3.14159265/180);
//  y = L1 * sin(theta1*3.14159265/180) + L2 * sin(theta1*3.14159265/180 + theta2*3.14159265/180); 
  theta1 = theta1*(3.14159265);
  theta1 = theta1/180;
  theta2 = theta2*(3.14159265);
  theta2 = theta2/180;
  x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
  y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
//  Serial.print("X axis reading: ");
//  Serial.print(x);
//  Serial.print(", Y axis reading: ");
//  Serial.println(y);

  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
 
  // Base encoder
  // Define kinematic parameters you may need
//  force1 = 0.2; // You will generate a force by simply assigning this to a constant number (in Newtons)
  // Calculate the jacobian
  float J[2][2];
  J[0][0] = -L1 * sin(theta1) - L2 * sin(theta1 + theta2);
  J[0][1] = -L2 * sin(theta1 + theta2);
  J[1][0] = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
  J[1][1] = L2 * cos(theta1 + theta2);

//   Print the Jacobian matrix
//Serial.println("Jacobian Matrix:");
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
//   Serial.print(J[i][j]);
//   Serial.print("\t");
    }
//    Serial.println();
  }

    // Transpose the Jacobian matrix
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      J_transpose[j][i] = J[i][j];
    }
    
  }
  
  // Print the transposed Jacobian matrix
//  Serial.println("Transposed Jacobian Matrix:");
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
//      Serial.print(J_transpose[i][j]);
//      Serial.print("\t");
    }
//    Serial.println();
  

  // Multiply matrix by scalar
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      J_transpose[i][j] *= force1;
    
  }
  memcpy(Tp1, J_transpose, sizeof(J_transpose));
}

  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************
  // Base motor
  #ifdef virtual_wall
  Serial.println(y);
  // kinematic parameters
  double x_wall1 = -0.07;    // position of the wall in (m)
  double k_wall1 = 400;        //wall stiffness always > 0

  // Collison check
  if (y < x_wall1){
    digitalWrite(solenoidPin, HIGH); //Switch Solenoid ON
    delay(10000); //Wait 1 Second
    digitalWrite(solenoidPin, LOW); //Switch Solenoid OFF
    delay(1000); //Wait 1 Second        
    force1 = k_wall1*(x_wall1 + y);
Serial.println("in wall");
//  Serial.println(force1,5);
  }
  else {
    force1 = 0;
  }
  #endif

  #ifdef A_Hard_Surface
  double A = 0.5;//amplitude, arbritrary value to scale velocity
  double lamda = -1.5; //decay rate
  double omega = 100; //frequency of sine wave
  double t_impact = 0; //IC of time and count to for the following few lines
  double tref = 0;
  double count = 0;
  double t = 0;

  if (xh > x_wall) //if in the wall, the counter will start counting
  {
    count = count +1;
    t_impact = millis()/1000; //records the time at which the handle hits the wall

    if (count == 20)//record time after an arbritrary amount of time inside the wall
    {
      tref = millis()/1000;
     }
     t = t_impact-tref;//time for decaying sinusoid
     force = k*(x_wall-xh)+A*vel*exp(lamda*t)*sin(omega*t);//force of the virtual wall, should feel like dropping a marble on the floor
     //as in the marble hitting the ground, bouncing once, and then stationary
  }
  
  #endif

 
  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************

  //base motor
  // Determine correct direction for motor torque
  if(force1 > 0) { 
    digitalWrite(dirPin1, LOW);
  } else {
    digitalWrite(dirPin1, HIGH);
  }
  // Set the motor speed to maximum
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      Tp1[i][j] = abs(Tp1[i][j]);
    }
  }

  duty1 = sqrt(Tp1[0][0] + Tp1[1][1] )/0.0183;

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
//  duty1 = sqrt(abs(Tp1)/0.0183);

  // Make sure the duty cycle is between 0 and 100%
  if (duty1 > 1) {            
    duty1 = 1;
  } else if (duty1 < 0) { 
    duty1 = 0;
  }  
  
  output1 = (int)(duty1* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin1,output1);  // output the signal
  }
}
// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency1(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

void setPwmFrequency2(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
