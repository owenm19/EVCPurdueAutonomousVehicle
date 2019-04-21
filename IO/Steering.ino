/**********
   Subsystem: Steering
   Description: Communication to steering motor controller and calculation of desired controller effort through encoder feedback
   Quadrature encoder currently uses only relative position.
 *********/
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3
#define STEERING_MOTOR_PIN 7
#define ENCODER_PWM_PIN A3

const uint16_t steerSpeedLimit = 300; //Range: 0-500. Overloads DC-DC or overheats motor if too high.
const int16_t zeroPosition = 2400; //Absolute position of encoder at zero steer angle
const int16_t positionRange = 1550; //Steering range in quadrature encoder positions (+/-)
const uint16_t kp = 250; //1000 x Proportional Response
const uint16_t ki = 0; //1000 x Integral Response
const uint16_t kd = 0; //1000 x Derivative Response

Servo steeringMotorController;
volatile uint32_t steerRiseTime; //For absolute encoder
volatile uint16_t startEncoderPos;
volatile int16_t encoderPos; //For Quadrature Encoder
bool A_set = false;
bool B_set = false;

void initSteering() {
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  pinMode(STEERING_MOTOR_PIN, OUTPUT);
  pinMode(ENCODER_PWM_PIN, INPUT);
  steeringMotorController.attach(STEERING_MOTOR_PIN);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), incrementencoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), incrementencoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PWM_PIN), evalRisingPWM, RISING);

  //Get absolute position
  steerRiseTime = 0;
  delay(10); //Lets the interrupts trigger at least once.
  //while(true){
  // Serial.println(startEncoderPos);
  //}
  detachInterrupt(digitalPinToInterrupt(ENCODER_PWM_PIN));
  startEncoderPos = zeroPosition; //Relative Mode
}

void writeSteering() {
  static int32_t errorInteg = 0;
  static int16_t lastError = 0;

  //Command byte to encoder ticks
  int16_t desiredEncoderPosition = map(desiredSteeringPosition, 0, 255, zeroPosition - (positionRange - 50), zeroPosition + (positionRange - 50));
  //Encoder ticks to communication value
  actualSteeringPosition = map(encoderPos + startEncoderPos, zeroPosition - (positionRange - 50), zeroPosition + (positionRange - 50), 0, 255);

  //PID
  int16_t error = (encoderPos + startEncoderPos) - desiredEncoderPosition;
  errorInteg += error;
  int16_t errorDeriv = error - lastError;
  int16_t pid = constrain((kp * error + ki * errorInteg + kd * errorDeriv) / 1000.0 + 1500, 1500 - steerSpeedLimit, 1500 + steerSpeedLimit);
  lastError = error;
  //steeringMotorController.writeMicroseconds(pid);

  //Serial.print(desiredEncoderPosition);
  //Serial.print(" , ");
  //Serial.println(encoderPos + startEncoderPos);
}

void incrementencoderA() {
  A_set = digitalRead(ENCODER_A_PIN) == 1;
  encoderPos += (A_set != B_set) ? +1 : -1;
}

void incrementencoderB() {
  B_set = digitalRead(ENCODER_B_PIN) == 1;
  encoderPos += (A_set == B_set) ? +1 : -1;
}

void evalRisingPWM() {
  steerRiseTime = micros();
  attachInterrupt(digitalPinToInterrupt(ENCODER_PWM_PIN), evalFallingPWM, FALLING);
}

void evalFallingPWM() {
  startEncoderPos = micros() - steerRiseTime;
  attachInterrupt(digitalPinToInterrupt(ENCODER_PWM_PIN), evalRisingPWM, RISING);
}