
#define ML_PIN1   4
#define ML_PIN2   5
#define ML_PIN3   6
#define ML_PIN4   7

#define MR_PIN1   8
#define MR_PIN2   9
#define MR_PIN3  10
#define MR_PIN4  11

#define BUTT_PIN  12

#define STEPS_PER_TURN 48
#define MIN_X   0
#define MAX_X 100 * STEPS_PER_TURN
#define MIN_Y   0
#define MAX_Y  50 * STEPS_PER_TURN


// bit - pin variables
unsigned char ml = 1,   // motor left pins
              mr = 1;   // motor right pins
// location variables
short x,  // planar position
      y;
int ll,       // length left
    lr,       // length right
    tgt_ll,   // target length left
    tgt_lr;   // target length right

void set_motor_pins() {
  digitalWrite(ML_PIN1, LOW);
  digitalWrite(ML_PIN2, LOW);
  digitalWrite(ML_PIN3, LOW);
  digitalWrite(ML_PIN4, LOW);
  
  digitalWrite(MR_PIN1, LOW);
  digitalWrite(MR_PIN2, LOW);
  digitalWrite(MR_PIN3, LOW);
  digitalWrite(MR_PIN4, LOW);

  if(ml & 1) digitalWrite(ML_PIN1, HIGH);
  if(ml & 2) digitalWrite(ML_PIN2, HIGH);
  if(ml & 4) digitalWrite(ML_PIN3, HIGH);
  if(ml & 8) digitalWrite(ML_PIN4, HIGH);

  if(mr & 1) digitalWrite(MR_PIN1, HIGH);
  if(mr & 2) digitalWrite(MR_PIN2, HIGH);
  if(mr & 4) digitalWrite(MR_PIN3, HIGH);
  if(mr & 8) digitalWrite(MR_PIN4, HIGH);
}

void step_right(unsigned char *motor) {
  *motor = *motor >> 1;
  if(*motor & 0x0F == 0) {
    *motor == 8;
  }
}
void step_left(unsigned char *motor) {
  *motor = *motor << 1;
  if(*motor & 0x0F == 0) {
    *motor == 1;
  }
}

void map_xy_to_lengths(int x, int y, int *ll, int *lr) {
  long xx = x * x;
  long yy = y * y;
  
  *ll = (int) sqrt(xx + yy);
  xx = (MAX_X - x) * (MAX_X - x);
  yy = (MAX_Y - y) * (MAX_Y - y);
  *lr = (int) sqrt(xx + yy);
}




short val1 = 0xFF;  // current value of X
short val2 = 0xFF;  // current value of Y
// warning: int is -32768..32767 on UNO!
long target_val1 = 0;
long target_val2 = 0;
double dval1 = (double)val1;  // current value of x in double and not int
double dval2 = (double)val2;  // current value of Y in double and not int
int caret_speed = 1;
double dstep1 = caret_speed;  // initial value of step X
double dstep2 = caret_speed;  // initial value of step Y

void set_target(int trg1, int trg2) {

  target_val1 = trg1;
  target_val2 = trg2;

  // set step so that both arrive the target at the same time
  // if distance1 > distance2 then step2 must be < speed ( = dist2/dist1 )
  // and vice versa.
  if(abs((double)target_val1 - dval1) > abs((double)target_val2 - dval2)) {
    dstep1 = ((double)target_val1 - dval1) / abs((double)target_val1 - dval1) * (double)caret_speed;
    dstep2 = ((double)target_val2 - dval2) / abs((double)target_val1 - dval1) * (double)caret_speed;
  }
  else
  if(abs((double)target_val2 - dval2) > 0) {
    dstep1 = ((double)target_val1 - dval1) / abs((double)target_val2 - dval2) * (double)caret_speed;
    dstep2 = ((double)target_val2 - dval2) / abs((double)target_val2 - dval2) * (double)caret_speed;
  }
  else {
    dstep1 = 0;
    dstep2 = 0;
  }

#ifdef DEBUG_DAC
  sprintf(outdbg, "set target: %lf %d   %lf %d", dval1, target_val1, dval2, target_val2);
  Serial.println(outdbg);
#endif
}

int step_ahead() {

    if(abs(dval1 - target_val1) > caret_speed + 1) { // pion
      dval1 += dstep1;
      val1 = (short) (dval1);
    }
    else {
      val1 = target_val1;
    }

    if(abs(dval2 - target_val2) > caret_speed + 1) { // poziom
      dval2 += dstep2;
      val2 = (short) (dval2);
    }
    else {
      val2 = target_val2;
    }

    return (val1 == target_val1 && val2 == target_val2);  // TRUE = target reached;
}




void setup() {
  pinMode(ML_PIN1, OUTPUT);
  pinMode(ML_PIN2, OUTPUT);
  pinMode(ML_PIN3, OUTPUT);
  pinMode(ML_PIN4, OUTPUT);

  pinMode(MR_PIN1, OUTPUT);
  pinMode(MR_PIN2, OUTPUT);
  pinMode(MR_PIN3, OUTPUT);
  pinMode(MR_PIN4, OUTPUT);

  pinMode(BUTT_PIN, INPUT);

  digitalWrite(ML_PIN1, LOW);
  digitalWrite(ML_PIN2, LOW);
  digitalWrite(ML_PIN3, LOW);
  digitalWrite(ML_PIN4, LOW);
  digitalWrite(MR_PIN1, LOW);
  digitalWrite(MR_PIN2, LOW);
  digitalWrite(MR_PIN3, LOW);
  digitalWrite(MR_PIN4, LOW);
  
  // put your setup code here, to run once:
  map_xy_to_lengths(MAX_X / 2, MAX_Y, &tgt_ll, &tgt_lr);
  map_xy_to_lengths(MAX_X / 2, MAX_Y, &ll, &lr);
}

void loop() {

  static unsigned int ii = 1;

  if(ii >= 5000) {
    ii = 0;
    x = random(MIN_X, MAX_X);
    y = random(MIN_Y, MAX_Y);
    map_xy_to_lengths(x, y, &tgt_ll, &tgt_lr);
    set_target(tgt_ll, tgt_lr);
  }

  // if i == 0 do not do anything; just wait for reaching the target
  if (ii > 0) {
    ii++;
  }
  if( step_ahead() ) { // TRUE = target reached
    if(ii == 0)
      ii = 1; // bump it to make it counting
  }

  if(val1 > ll)
    step_right(&ml);
  else if(val1 < ll)
    step_left(&ml);

  if(val2 > lr)
    step_right(&mr);
  else if(val2 < mr)
    step_left(&mr);

  set_motor_pins();

  delay(1);
}
