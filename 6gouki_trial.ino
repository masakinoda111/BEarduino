#include <Wire.h>
#include <SparkFun_MMA8452Q.h>

#define Pi 3.1416

#define LPin1  7    // 左のモータ（順回転でHIGH、逆回転でLOW）
#define LPin2  8    // 左のモータ（順回転でLOW、逆回転でHIGH）
#define LSpeedPin 6 // モーターの回転速度を調整するためのピン

#define RPin1  2    // 右のモータ（順回転でHIGH、逆回転でLOW）
#define RPin2  4    // 右のモータ（順回転でLOW、逆回転でHIGH）
#define RSpeedPin 5 // モーターの回転速度を調整するためのピン

/* setAngleのパラメータ */
#define RESET_ANGLE_SPEED_MAX 170 // 回転速度の設定
#define RESET_ANGLE_SPEED_MIN 120
#define TURN_CENTER 0.1

/* moveStraight_PIDのパラメータ */
#define FORWARD_SPEED 255   // 直進速度、3Vは75くらい
#define BACKWARD_SPEED 175

/* ローパスフィルタのパラメータ */
#define K 0.05   // 0.0~1.0

/* PID制御のパラメータ */
#define Kp 3
#define Ki 0.4
#define Kd 0.2

/* 閾値 */
#define DIST_THRESHOLD 50
#define RESET_ANGLE_THRESHOLD 2

/* センサー定義 */
MMA8452Q accel;

/* ローパスフィルタの過去データ配列 */
// {現在の値, 単位時間前の値}
float x_g_array[2] = {0.0, 0.0};
float y_g_array[2] = {0.0, 0.0};
float dist_array[2] = {0.0, 0.0};

/* PID制御の過去データ配列 */
// {現在の値, 単位時間前の値, 2単位時間前の値}
float angle_dif_array_ms[3] = {0.0, 0.0, 0.0};
float angle_dif_array_t[3] = {0.0, 0.0, 0.0};

/* カウント用 */
int i;

void software_reset() {
  asm volatile ("jmp 0");
}

/* モーターにスピードと方向(+:順方向,-:逆方向)を送る */
void driveMotors(int L_speed, int R_speed) { 
  if(L_speed > 255) L_speed = 255;
  if(L_speed < -255) L_speed = -255;
  if(R_speed > 255) R_speed = 255;
  if(R_speed < -255) R_speed = -255;

  if(L_speed>0) {
  digitalWrite(LPin1,HIGH);
  digitalWrite(LPin2,LOW);
  analogWrite(LSpeedPin,L_speed);
  } else {
  digitalWrite(LPin1, LOW);
  digitalWrite(LPin2,HIGH);
  analogWrite(LSpeedPin,-L_speed);
  }

  if(R_speed>0) {
  digitalWrite(RPin1,HIGH);
  digitalWrite(RPin2,LOW);
  analogWrite(RSpeedPin,R_speed);
  } else {
  digitalWrite(RPin1,LOW);
  digitalWrite(RPin2,HIGH);
  analogWrite(RSpeedPin,-R_speed);
  }
}

/* モーターの回転を急停止 ..._direction:順回転は1, 逆回転は-1*/
void stopMotors(int L_direction, int R_direction) {
  driveMotors(L_direction*255, R_direction*255);
  delay(100);
  digitalWrite(LPin1,LOW);
  digitalWrite(LPin2,LOW);
  digitalWrite(RPin1,LOW);
  digitalWrite(RPin2,LOW);
  Serial.println("MOTOR STOP");
}

/* ローパスフィルタ（ノイズ除去）入力値で配列を更新 */
void LPF(float val, float *vec) {
  vec[0] = val;
  vec[0] = (1-K)*vec[1] + K*vec[0];
  vec[1] = vec[0];
}

/* PID制御 入力値で配列を更新 */
float PID(float val, float *vec) {
  static float u;
  vec[0] = val;
  u = Kp*vec[0] + Ki*(vec[0]+vec[1]+vec[2]) + Kd*(vec[0]-vec[1]);
  vec[2] = vec[1];
  vec[1] = vec[0];
  return u;
}

/* 速度変換関数 for PID */
int RSpeedFunc(int e, int c_max) {
  static int c;
  c = e * (-1) + c_max;
  if(c > c_max) c = c_max;
  if(c < 0) c = 0;
  return c;
}

int LSpeedFunc(int e, int c_max) {
  static int c;
  c = e + c_max;
  if(c > c_max) c = c_max;
  if(c < 0) c = 0;
  return c;
}

int turnSpeedFunc(int e, int c_max, int c_min) {
  static int c;
  static int direction;
  if(e > 0) {
    direction = -1;
  } else {
    direction = 1;
  } 
  e = -direction*e;
  c = e + c_min;
  if(c > c_max) c = c_max;
  return direction*c;
}

/* LPFを通して、目標角度と現在の角度との差を計算（-180~180[deg]）*/
float getAngleDif(float target_angle) {
  static float x_g=0.0;
  static float y_g=0.0;
  static float current_angle=0.0;
  static float angle_dif=0.0;

  LPF(accel.getX(), x_g_array);
  LPF(accel.getY(), y_g_array);
  current_angle = atan2(y_g_array[0], x_g_array[0])*180/Pi;
  angle_dif = current_angle - target_angle;

  // -180~180[deg]内に収める
  if(angle_dif > 180) angle_dif -= 360;
  if(angle_dif < -180) angle_dif += 360;

//  Serial.print("current angle = ");
//  Serial.println(current_angle);
//  Serial.print("difference    = ");
//  Serial.println(angle_dif);

  return angle_dif;
}

/* LPFを通して、測距センサーの値を計算 */
float getDistVal() {
  LPF(analogRead(A0), dist_array);
  return dist_array[0];
}

/* 左右に転回 speed:反時計回りを正 center:転回中心(L:-1 ~ O:0 ~ R:1) */
void turn(int speed, float center) {
  static int out_L=0;
  static int out_R=0;
  if(center > 0) {
    out_L = speed;
    out_R = -speed * (1-center) / (1+center);
  } else {
    out_L = -speed * (1-center) / (1+center);
    out_R = speed;
  }
  driveMotors(out_L, out_R);

  if(speed > 0) {
    Serial.println("TURN LEFT");
  } else {
    Serial.println("TURN RIGHT"); 
  }
}

/* 直進（PID制御）*/
void moveStraight(float target_angle, boolean forward) {
  static int out_L=0;
  static int out_R=0;
  static float u=0;
  static float pre_dist=0.0;
  static float cur_dist=0.0;
  int cnt=0;

  Serial.println("--------In moveStraight--------");

  // if(forward) for(i=0;i<10;i++) pre_dist = getDistVal();

  while(1) {
    digitalWrite(13, LOW);

    u = PID(getAngleDif(target_angle), angle_dif_array_ms);  // LPF込み
    if(forward) {
      out_L = LSpeedFunc(u, FORWARD_SPEED);
      out_R = RSpeedFunc(u, FORWARD_SPEED);
     Serial.println("GO FORWARD");
    } else {
      out_R = -1*LSpeedFunc(u, BACKWARD_SPEED);
      out_L = -1*RSpeedFunc(u, BACKWARD_SPEED);
      Serial.println("GO BACKWARD");
    }
    driveMotors(out_L,out_R);
    delay(100);

    
    // センサーの距離が変わるとループを抜ける
    /*
    cur_dist = getDistVal();
    Serial.println(cur_dist-pre_dist);
    if(abs(cur_dist-pre_dist) > DIST_THRESHOLD) {
      stopMotors(-1, -1);
      digitalWrite(13, HIGH);
      Serial.println("距離が変わりました");
      break;
    }
    */
  }
  Serial.println("--------Out of moveStraight--------");
}

/* 目標角度へ転回 direction:反時計回りなら1, 時計回りなら-1（Rの回転方向）*/ 
void setAngle(float target_angle, float center, int direction) {
  static float angle_dif=0.0;
  static float speed=0.0;
  static int out_L=0;
  static int out_R=0;
  static float u=0;

  Serial.println("--------In setAngle--------");
  
  Serial.println("--------Out of setAngle--------");
}

/* 繰り返し部分の動作順を記述 */
void doErase() {
  static float pre_dist=0.0;
  static float cur_dist=0.0;

  while(1) {
    setAngle(0, 0, -1);

    moveStraight(0, true);

    setAngle(180, 0, -1);

    moveStraight(180, true);

    setAngle(0, 0, 1);
    
  }
}

void returnHome() {
  moveStraight(-90, false);
  setAngle(0, 0, -1);
  delay(10000);
}

/* 加速度センサーの設定 */
void setupMMA8452() {
  // シリアルポートを9600 bps[ビット/秒]で初期化
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Adafruit MMA8452");
  if (! accel.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8452 found!");
}

/* 入出力ピンの設定など */
void setup() {
  pinMode(LPin1, OUTPUT);
  pinMode(LPin2, OUTPUT);
  pinMode(LSpeedPin, OUTPUT);
  pinMode(RPin1, OUTPUT);
  pinMode(RPin2, OUTPUT);
  pinMode(RSpeedPin, OUTPUT);

  // 加速度センサを使うため
  setupMMA8452();

  // 測距センサ
  pinMode(A0, INPUT);
  pinMode(13, OUTPUT); // LED点灯用
}

void test(){
  for(i=0; i<255; i++){
    driveMotors(-i, -i);
    Serial.println(i);
    delay(100);
  }
}

/* メインループ */
void loop() {
  doErase();
//  returnHome();
//  test();
}
