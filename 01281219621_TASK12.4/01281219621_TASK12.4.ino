// PID class definition
class PID {
  private:
    double Kp, Ki, Kd;
    double integral, previousError, deltaTime;
    double integralLimit = 100;

  public:
    // Constructor to initialize PID constants and variables
    PID(double p, double i, double d, double dt) {
      Kp = p;
      Ki = i;
      Kd = d;
      deltaTime = dt;
      integral = 0;
      previousError = 0;
    }

    // Function to calculate the PID output
    double calculate(double setpoint, double input) {
      double error = setpoint - input;        // Calculate error
      double Pout = Kp * error;               // Proportional term

      integral += error * deltaTime;          // Integral term
      if (integral > integralLimit) integral = integralLimit;
      if (integral < -integralLimit) integral = -integralLimit;
      double Iout = Ki * integral;

      double derivative = (error - previousError) / deltaTime;  // Derivative term
      double Dout = Kd * derivative;

      previousError = error;  // Save error for next loop

      return Pout + Iout + Dout;  // Return the PID output
    }
};

// Soft-start function using Exponential Smoothing Filter
double softStart(double target, double current, double alpha) {
  return alpha * target + (1 - alpha) * current;
}

// Pin definitions
const int motorPinPWM = 10;  // PWM pin for motor speed control
const int motorPin1 = 8;     // H-bridge control pin 1
const int motorPin2 = 9;     // H-bridge control pin 2
const int potPin = A0;       // Potentiometer pin for speed input

// PID controller setup: (Kp, Ki, Kd, deltaTime)
PID motorPID(2.0, 5.0, 1.0, 0.1);  // Tuning the PID controller 

// Variables
double setpoint = 0;           // Desired motor speed
double motorSpeed = 0;         // Simulated motor speed
double output = 0;             // PID output
double previousOutput = 0;     // To hold the previous output for smoothing

void setup() {
  pinMode(motorPinPWM, OUTPUT);   // Motor PWM output pin
  pinMode(motorPin1, OUTPUT);     // Motor control pin 1
  pinMode(motorPin2, OUTPUT);     // Motor control pin 2
  pinMode(potPin, INPUT);         // Potentiometer input pin
  Serial.begin(9600);            
}

void loop() {
  // Read potentiometer value and map it to 0-255 range for motor speed control
  setpoint = analogRead(potPin) * 255.0 / 1023.0;

  // Simulate motor speed change (could be actual speed reading if using an encoder)
  motorSpeed += (setpoint - motorSpeed) * 0.1;

  // Calculate the PID output based on the setpoint and current motor speed
  output = motorPID.calculate(setpoint, motorSpeed);
  output = constrain(output, 0, 255);  // Ensure the output is within 0-255 range

  // Apply soft-start to the PID output
  double smoothedOutput = softStart(output, previousOutput, 0.1);

  // Motor control (for forward direction)
  digitalWrite(motorPin1, HIGH);  // Motor direction control
  digitalWrite(motorPin2, LOW);
  analogWrite(motorPinPWM, smoothedOutput);  // Set motor speed

  // Update previous output for the next loop iteration
  previousOutput = smoothedOutput;

  // Print values to Serial monitor
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" Motor Speed: ");
  Serial.print(motorSpeed);
  Serial.print(" PID Output: ");
  Serial.println(smoothedOutput);

  delay(100);  
