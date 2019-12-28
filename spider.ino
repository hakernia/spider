
#define ML_PIN1   4
#define ML_PIN2   5
#define ML_PIN3   6
#define ML_PIN4   7

#define MR_PIN1   8
#define MR_PIN2   9
#define MR_PIN3  10
#define MR_PIN4  11

#define BUTTON_LEFT_PIN  12
#define BUTTON_RIGHT_PIN  A0

#define LED_PIN  13

#define STEPS_PER_TURN 48
#define OFF_X   200
#define MAX_X  3000   // 100 * STEPS_PER_TURN
#define MIN_Y     0
#define MAX_Y  1100   //  50 * STEPS_PER_TURN


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
  // energize new pin first, not to leave unenergized pins any time
  if(ml & 1) digitalWrite(ML_PIN1, HIGH);
  if(ml & 2) digitalWrite(ML_PIN2, HIGH);
  if(ml & 4) digitalWrite(ML_PIN3, HIGH);
  if(ml & 8) digitalWrite(ML_PIN4, HIGH);

  if(mr & 1) digitalWrite(MR_PIN1, HIGH);
  if(mr & 2) digitalWrite(MR_PIN2, HIGH);
  if(mr & 4) digitalWrite(MR_PIN3, HIGH);
  if(mr & 8) digitalWrite(MR_PIN4, HIGH);

  // deenergize other pins
  if(!(ml & 1)) digitalWrite(ML_PIN1, LOW);
  if(!(ml & 2)) digitalWrite(ML_PIN2, LOW);
  if(!(ml & 4)) digitalWrite(ML_PIN3, LOW);
  if(!(ml & 8)) digitalWrite(ML_PIN4, LOW);
  
  if(!(mr & 1)) digitalWrite(MR_PIN1, LOW);
  if(!(mr & 2)) digitalWrite(MR_PIN2, LOW);
  if(!(mr & 4)) digitalWrite(MR_PIN3, LOW);
  if(!(mr & 8)) digitalWrite(MR_PIN4, LOW);
}

void step_right(unsigned char *motor) {
  *motor = ((*motor) >> 1);
  if(((*motor) & 0x0F) == 0) {
    *motor = 8;
  }
}
void step_left(unsigned char *motor) {
  *motor = ((*motor) << 1);
  if(((*motor) & 0x0F) == 0) {
    *motor = 1;
  }
}

void map_xy_to_lengths(int x, int y, int *ll, int *lr) {
  long xx = sq((long)x);
  long yy = sq((long)y);
/*
  Serial.print("x,y= ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);

  Serial.print("  xx,yy= ");
  Serial.print(xx);
  Serial.print(", ");
  Serial.print(yy);
*/
  *ll = (int) sqrt(xx + yy);
  xx = sq((long)((long)MAX_X+2*OFF_X - (long)x));
  // yy = (MAX_Y - y) * (MAX_Y - y);  // just x is reversed on the right motor
  *lr = (int) sqrt(xx + yy);

//  *ll = *ll * log(*ll/100 + 0.5);
//  *lr = *lr * log(*lr/100 + 0.5);
/*
  Serial.print("  ll,lr= ");
  Serial.print((int)*ll);
  Serial.print(", ");
  Serial.println((int)*lr);
*/
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

void init_target_reached() {
  dval1 = target_val1;
  dval2 = target_val2;
  val1 = dval1;
  val2 = dval2;
  dstep1 = 0;
  dstep2 = 0;
}

/*
 * Set target_val1, target_val2 and dstep1, dstep2
 * The dsteps are then used by step_ahead() to reach target vals.
 */
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

  pinMode(BUTTON_LEFT_PIN, INPUT);
  pinMode(BUTTON_RIGHT_PIN, INPUT);
  
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(ML_PIN1, LOW);
  digitalWrite(ML_PIN2, LOW);
  digitalWrite(ML_PIN3, LOW);
  digitalWrite(ML_PIN4, LOW);
  digitalWrite(MR_PIN1, LOW);
  digitalWrite(MR_PIN2, LOW);
  digitalWrite(MR_PIN3, LOW);
  digitalWrite(MR_PIN4, LOW);
  
  // put your setup code here, to run once:
  set_target(MAX_X / 2 + OFF_X, MAX_Y);
  map_xy_to_lengths(MAX_X / 2 + OFF_X, MAX_Y, &tgt_ll, &tgt_lr);
  ll = tgt_ll;
  lr = tgt_lr;
  init_target_reached();

  Serial.begin(9600);

  delay(3000);
}

int targets_x[] = {OFF_X,
                   OFF_X + MAX_X/2,
                   OFF_X + MAX_X/2,
                   OFF_X + MAX_X/2,
                   MAX_X,
                   OFF_X,
                   OFF_X + MAX_X/2,
                   MAX_X,
                   MAX_X };
int targets_y[] = {MAX_Y,
                   MAX_Y,
                   MIN_Y,
                   MAX_Y,
                   MAX_Y,
                   MIN_Y,
                   MIN_Y,
                   MIN_Y,
                   MAX_Y };

void loop() {

  static unsigned int ii = 1;
  static unsigned char target_num = 0;

  if(ii >= 500) {
    ii = 0;
    x = OFF_X + random(MAX_X);
    y = random(MIN_Y, MAX_Y);

    x = targets_x[target_num];
    y = targets_y[target_num];
    if(target_num < 8)
        target_num++;
    else
        target_num = 0;
        
    //map_xy_to_lengths(x, y, &tgt_ll, &tgt_lr);
    //set_target(tgt_ll, tgt_lr);
    set_target(x, y);
    /*
    Serial.print("targets: (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(") => ");
    Serial.print(tgt_ll);
    Serial.print(", ");
    Serial.println(tgt_lr);
    */
  }

  // if i == 0 do not do anything; just wait for reaching the target
  if (ii > 0) {
    ii++;
  }
  if( step_ahead() ) { // TRUE = target reached
    if(ii == 0)
      ii = 1; // bump it to make it counting
    //Serial.print("step_ahead() == TRUE   ii == ");
    //Serial.println(ii);
  } else {
    //Serial.print("step_ahead() == FALSE   ii == ");
    //Serial.println(ii);
    map_xy_to_lengths(val1, val2, &tgt_ll, &tgt_lr);
  }

  if(ll < tgt_ll) {
      step_left(&ml);
      ll++;
  } else 
  if(ll > tgt_ll) {
      step_right(&ml);
      ll--;
  }

  if(lr < tgt_lr) {
      step_left(&mr);
      lr++;
  } else
  if(lr > tgt_lr) {
      step_right(&mr);
      lr--;
  }

/*
  if((val1 > ll) || digitalRead(BUTTON_LEFT_PIN)) {
//    step_right(&ml);
    step_left(&ml);
    ll = (int)val1;
  }
  else if(val1 < ll) {
//    step_left(&ml);
    step_right(&ml);
    ll = (int)val1;
  }

  if((val2 > lr) || digitalRead(BUTTON_RIGHT_PIN)) {
//    step_right(&mr);
    step_left(&mr);
    lr = (int)val2;
  }
  else if(val2 < lr) {
//    step_left(&mr);
    step_right(&mr);
    lr = (int)val2;
  }
*/


/*
  Serial.print("val1,val2 = ");
  Serial.print((int)val1);
  Serial.print(", ");
  Serial.println((int)val2);
  Serial.print("    ll,lr = ");
  Serial.print((int)ll);
  Serial.print(", ");
  Serial.println((int)lr);
  Serial.print("    ml,mr = ");
  Serial.print(ml);
  Serial.print(", ");
  Serial.println(mr);
*/
  set_motor_pins();

  delay(12);
}
