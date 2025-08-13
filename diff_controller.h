typedef struct {
  double TargetTicksPerFrame;
  long Encoder;
  long PrevEnc;
  int PrevInput;
  int ITerm;
  long output;
} SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 8;
int Kd = 5;
int Ki = 2;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

void resetPID() {
  leftPID.TargetTicksPerFrame = 0.0;
  leftPID.Encoder = readEncoder(0);
  leftPID.PrevEnc = leftPID.Encoder;
  leftPID.output = 0;
  leftPID.PrevInput = 0;
  leftPID.ITerm = 0;

  rightPID.TargetTicksPerFrame = 0.0;
  rightPID.Encoder = readEncoder(1);
  rightPID.PrevEnc = rightPID.Encoder;
  rightPID.output = 0;
  rightPID.PrevInput = 0;
  rightPID.ITerm = 0;
}

void doPID(SetPointInfo * p) {
  int input = p->Encoder - p->PrevEnc;
  long Perror = p->TargetTicksPerFrame - input;

  // DO NOT accumulate output
  long output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;

  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
  p->PrevEnc = p->Encoder;
}

void updatePID() {
  leftPID.Encoder = readEncoder(0);
  rightPID.Encoder = readEncoder(1);

  int leftMeasured = leftPID.Encoder - leftPID.PrevEnc;
  int rightMeasured = rightPID.Encoder - rightPID.PrevEnc;

  if (!moving) {
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  doPID(&rightPID);
  doPID(&leftPID);
  setMotorSpeeds(leftPID.output, rightPID.output);

  Serial.print("Target: ");
  Serial.print(leftPID.TargetTicksPerFrame);
  Serial.print(" | Measured: ");
  Serial.print(leftMeasured);
  Serial.print(" | Output: ");
  Serial.println(leftPID.output);
}

