#include <SparkFun_MMA8452Q.h>
#include <Wire.h>

#define Pi 3.1416

/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.*/
//Define L298N Dual H-Bridge Motor Controller Pins

#define dir1PinL  7    //左のモータ（順回転でHIGH、逆回転でLOW）
#define dir2PinL  8    //左のモータ（順回転でLOW、逆回転でHIGH）
#define speedPinL 6    //モーターの回転速度を調整するためのピン。Pulse Width Modulation

#define dir1PinR  2    //右のモータ（順回転でHIGH、逆回転でLOW）
#define dir2PinR  4    //右のモータ（順回転でLOW、逆回転でHIGH）
#define speedPinR 5    //モーターの回転速度を調整するためのピン。Pulse Width Modulation

//測距センサ、障害物センサ
#define forwardPin  0   //前の測距センサ
#define backPin  1   //後ろの障害物センサ

/*調整するパラメータ --------------------------------------------------------------------------- */
//重心のずれ
#define D_CENTER 10

//測距センサのパラメータ
#define ACmeter_count 10   //過去何回分の加速度センサーの値の平均値を信用ある値として使うか
#define BUF_SIZE 8
#define SOKKYO_THRESHOLD 0.075

//加速度センサのパラメータ
/* ローパスフィルタのパラメータ */
#define K 0.05

//set_angleのパラメータ：
#define SET_ANGLE_SPEED 180 //回転速度の設定
#define SET_ANGLE_THRESHOLD 10   //set_angleでの許容幅
#define E0 0
#define E1 3
#define E2 0          //移動時間の設定：keep((1+E0*cos(d_angle))*(E1*abs(d_angle) + E2))

//go_Forward_Feedbackのパラメータ：
#define FORWARD_SPEED 255   //直進速度、255で7.2Vくらい
#define FORWARD_DELAY 5     //移動時間の設定。向いている方向が正しければ "3*FORWARD_DELAY" ms進む、ずれていれば "FORWARD_DELAY" ms進む

//go_Backward_Feedbackのパラメータ：
#define BACK_SPEED 150 //255
#define BACK_DELAY 3

// PID制御のパラメータ  e0 = d_angle; u = Kp*e0 + Ki*(e0+e1+e2) + Kd*(e0-e1);
#define Kp 0.04
#define Ki 0
#define Kd 0

float calculate_mean(float *buf){
  float sum = 0.0;
  for (int i = 0; i < BUF_SIZE; i++){
    sum += buf[i];
  }
  return sum/BUF_SIZE;
}

MMA8452Q accel;                   // create instance of the MMA8452 class

/***************motor control***************/

/*---------------------------------------*/
float x_g_array[2] = {0.0, 0.0};
float y_g_array[2] = {0.0, 0.0};
//ローパスフィルタ
void LPF(float val, float *vec){
  vec[0] = (1-K)*vec[1] + K*val;
  vec[1] = vec[0];
}
/*---------------------------------------*/


void software_reset() {
  asm volatile ("  jmp 0");
}

/**************加速度センサのため*************/
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

  //mma.setRange(MMA8452_RANGE_2_G);
//
//  Serial.print("Range = ");
//  Serial.print(2 << mma.getRange());
//  Serial.println("G");
}
/************加速度センサのため***************/

/***********加速度センサからの入力に応じた処理***********/

  /*ここに、gForceX, gForceY, gForceZの値に応じた処理を書く*/
  /*この加速度センサはx、y、z軸いずれも、鉛直上を向いているときに、1000を出力。*/
  /*姿勢は、車が鉛直壁に張り付いているとして、z軸方向が壁の法線方向と考えられるので、加速度のx軸、y軸成分(gForceX, gForeceY)を読み取り、それらの変化から姿勢を制御することを目標とする。*/

//現在の角度を測定

float gForceX, gForceY, gForceZ; //↑を重力加速度の何倍か、に変換した値

float current_angle; //現在の機体が向いている方向。前はmoving_angleという名前だった。-180~180度。


void getAngle(){  //読み取ったgの値から機体が向いている方向を計算して、current_angleに代入する
  LPF(accel.getX(), x_g_array);
  LPF(accel.getY(), y_g_array);

  gForceX = x_g_array[0];
  gForceY = y_g_array[0];

  //水平右方向を０度とするロボットの向いている角度。範囲は-180度～180度
  current_angle= atan2(gForceY*(-1), gForceX*(-1))*180/Pi;
  printData() ;
}

void getAngle_0(){  //読み取ったgの値から機体が向いている方向を計算して、current_angleに代入する
  gForceX = 0;
  gForceY = 0;
  for(int i=0;i<ACmeter_count;i++){ //振動による計測のブレが大きいので、過去ACmeter_count回の平均をとって現在の加速度の値とする
    if (accel.available()) {      // Wait for new data from accelerometer
      gForceX += accel.getX();
      gForceY += accel.getY();
    }
  }
  gForceX /= (ACmeter_count*1000);//読み取った値をgに変換
  gForceY /= (ACmeter_count*1000);

  //水平右方向を０度とするロボットの向いている角度。範囲は-180度～180度
  current_angle= atan2(-gForceY, -gForceX)*180/Pi;

  //Serial.print("    current_angle,gForceX,gForceY = ");
  //Serial.println(current_angle );
  //Serial.println(gForceX );
  //Serial.println(gForceY );
  //Serial.print("  \n");
}
void printData() {//シリアルモニタに出力
  //Serial.print("  gForce");
  //Serial.print(" X=");
  //Serial.print(gForceX);
  //Serial.print('g');
  //Serial.print(" Y=");
  //Serial.print(gForceY);
  //Serial.print('g');
//  Serial.print(" Z=");
//  Serial.print(gForceZ);
//  Serial.println('g');
  Serial.print("    current_angle = ");
  Serial.print(current_angle);
  Serial.print("  \n");
}

//current_angleとgoal_angleの差を求める。そのままだと差の絶対値が180を超えてしまう（正しい向きに回転しなくなるor激しく回転してしまうので困る）ので、差を-180～180の間に収める
float calculate_d_angle(float goal_angle){
  getAngle();
  float d_angle = current_angle - goal_angle;
  if(abs(d_angle)>180){
    if (d_angle>0){ d_angle-=360; }
    else { d_angle+=360; }
  }
  return d_angle;
}

float calculate_d_angle_0(float goal_angle){
  getAngle_0();
  float d_angle = current_angle - goal_angle;
  if(abs(d_angle)>180){
    if (d_angle>0){ d_angle-=360; }
    else { d_angle+=360; }
  }
  return d_angle;
}

//測距センサ
float voltage_buf[BUF_SIZE] = {};
int buf_counter;
int buf_flag;
int tmp_val;
float tmp_voltage;
float sum_of_tmp_voltage;
float mean_of_tmp_voltage;

void reset_dist() {
	tmp_val = 0;
	tmp_voltage = 0.0;
	sum_of_tmp_voltage = 0.0;
	mean_of_tmp_voltage = 0.0;
	buf_counter = 0;
	buf_flag = 0;
}

bool check_dist(int pin) {
	//voltage_bufに100回測った平均を記録していく
	for (int i = 0; i < 100; i++) {
		tmp_val = analogRead(pin);
		tmp_voltage = (tmp_val * 5) / 1024.0;
		sum_of_tmp_voltage += tmp_voltage;
	}
	mean_of_tmp_voltage = sum_of_tmp_voltage / 100;
	voltage_buf[buf_counter] = mean_of_tmp_voltage;
	buf_counter++;
	if (buf_counter == BUF_SIZE) {
		buf_counter = 0;
	}

	//BUF_SIZE回以上測定していて
	//かつbuf_size前の電圧と今の電圧の差が大きければ障害物があったとみなす
	if (buf_flag >= BUF_SIZE) {
		if((voltage_buf[buf_counter + 1] - mean_of_tmp_voltage) > SOKKYO_THRESHOLD) {
		return 1;
		}
	}
	else {
		buf_flag++;
	}
	return 0;
}


//モーターの操作
void drive_motors(int L_speed, int R_speed)  //モーターに送るspeed（電圧）と方向を指示する
{
  if(L_speed>=0){
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL,LOW);
  analogWrite(speedPinL,L_speed);
  }
  else{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,HIGH);
  analogWrite(speedPinL,-L_speed);
  }

  if(R_speed>=0){
  digitalWrite(dir1PinR,HIGH);
  digitalWrite(dir2PinR,LOW);
  analogWrite(speedPinR,R_speed);
  }
  else{
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,HIGH);
  analogWrite(speedPinR,-R_speed);
  }
}

void go_Forward(int speed_var, int goal_angle) //フィードバック制御なし、ただまっすぐ進もうとする
{
  drive_motors(speed_var + D_CENTER*cos(goal_angle), speed_var - D_CENTER*cos(goal_angle));  //第二項はそれぞれ重心のずれの補正分（D_CENTERで調整）
  //Serial.print("  GO FORWARD\n");
}

void go_Back(int speed_var, int goal_angle)  //後ろに進む（注意！goal_angleは機体を向かせたい方向なので、後ろに進む場合進行方向とは真逆）
{
  drive_motors(-speed_var + D_CENTER*cos(goal_angle), -speed_var - D_CENTER*cos(goal_angle));
  //Serial.print("  GO BACK\n");
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
  drive_motors(out_L, out_R);

  if(speed > 0) {
    Serial.println("TURN LEFT");
  } else {
    Serial.println("TURN RIGHT"); 
  }
}
void stop_Stop()    //モーターの回転を停止させる
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,LOW);
  //Serial.print("  STOP!\n");
}

void keep(int ms)  //msミリ秒だけ現在の電圧をキープ、そして停止する。前はmovementという関数だった
{
  delay(ms);
  stop_Stop();
}

//指定した角度に方向を直す。goal_angleが角度の目標値。goal_angleは-180度～180度の範囲で指定
void set_angle(float goal_angle){  
  float d_angle;
    d_angle = calculate_d_angle_0(goal_angle);
  while (abs(d_angle) > SET_ANGLE_THRESHOLD) {

	  //目標角度に修正
	  if (d_angle < 0) {
		  turn(SET_ANGLE_SPEED,0); //回転スピードの設定
	  }
	  else {
		  turn(SET_ANGLE_SPEED*(-1),0); //回転スピードの設定
	  }
   
	  keep((1 + E0 * cos(d_angle))*(E1*abs(d_angle) + E2)) ;//回転時間の設定
    d_angle = calculate_d_angle_0(goal_angle);
	  Serial.print("SET ANGLE: ");
	  Serial.println(goal_angle);
	  Serial.print("\n");
  }
}

int rightw_command(int e, int c_middle)
{
  int c;
  c = e * (-1) + c_middle;
  if(c > 255){
    c = 255;
  }
  if(c < 150){
    c = 150;
  }
  return c;
}

int leftw_command(int e, int c_middle)
{
  int c;
  c = e + c_middle;
  if(c > 255){
    c = 255;
  }
  if(c < 150){
    c = 150;
  }
  return c;
}

//制御

//フィードバック制御あり。
//角度を常に一定にキープしながら進むことができる。
//move_timeは移動時間。
//両輪の平均がspeedとなる
void go_Feedback(int goal_angle, int move_time, int speed) 
{
  set_angle(goal_angle);
	float e1 = 0; // 単位時間前の偏差
	float e2 = 0; // 2単位時間前の偏差
	float e0;
	int u;
	int outL = speed, outR = speed; //左右のモーターに出力される値
	float time_count = 0;  //time_countは経過した時間。これがmove_timeを下回っている間、while loopが回り続ける
  reset_dist();
	while (time_count < move_time) {
		//以降、d_angleを0に収束させるフィードバック制御：
		float d_angle = calculate_d_angle_0(goal_angle);
		if (abs(d_angle) > 30) {  //あまりに違う方向を向いている場合はその場で回転して角度を修正
			//端に乗り上げているケースが多いので一度バック
			drive_motors(-speed, -speed);
			keep(300);
			set_angle(goal_angle);
			e1 = 0; 
			e2 = 0; 
		}
		else  // 以下PID制御
		{
			e2 = e1;
			e1 = e0;
			e0 = d_angle;
			u = Kp * e0 + Ki * (e0 + e1 + e2) + Kd * (e0 - e1);
			outL += u;
			outR -= u;
			drive_motors(outL, outR);
     if(speed>0){
			if (check_dist(forwardPin) == 1)break;
     }
     else{
      if (check_dist(backPin) == 1)break;
     }
		}
	}
}

//消去アルゴリズム
void do_erase_all(){
//
//  //(0)まず、何往復かを特定
//  int i;
//  int num_of_roundtrip =3;
//  //num_of_roundtrip = measure_vertical_length(89, 20000);
//  drive_motors(255,0);
//  delay(100);
//  set_angle(2);

  //(1)次にnum_of_roundtrip分だけ往復運動を左上から開始
  go_Feedback(0, 10000, 250);
  for(int i=0;i<1;i++){
    drive_motors(-155,155);
    delay(10);
    set_angle(0);
    go_Feedback(0, 1000, 250);
  
    drive_motors(155,-155);
    delay(10);
    set_angle(180);
    go_Feedback(180, 1000, 250);
  }
  //(2)右端に移動して
    drive_motors(155,-155);
    delay(10);
    set_angle(0);
    go_Feedback(0, 1000, 150);

  //(3)右下に移動
  drive_motors(-255,0);
  delay(100);
  set_angle(89);
  go_Feedback(89,20000, 150);
  drive_motors(0,255);
  delay(100);
  set_angle(178);

  //(4)再びnum_of_roundtrip分だけ往復運動を左上から開始。ただし今回は上がる。
  for(int i=0;i<1;i++){
  go_Feedback(180, 1000, 250);
  drive_motors(-255,-255);
  delay(500);
  drive_motors(255,0);
  delay(1800);
  go_Feedback(0, 1000, 250);
  drive_motors(-255,-255);
  delay(500);
  drive_motors(0,255);
  delay(1800);
  }

  //(5)左端に移動
  drive_motors(255,255);
  delay(100);
  go_Feedback(178,20000, 250); //左にまっすぐ進む。int goal_angle, int move_time

  //左上に移動
  drive_motors(255,0);
  delay(100);
  set_angle(89);
  go_Feedback(89,20000, 250);
  drive_motors(255,0);
  delay(100);
  set_angle(0);
}



void setup()
{
  pinMode(dir1PinL, OUTPUT);
  pinMode(dir2PinL, OUTPUT);
  pinMode(speedPinL, OUTPUT);
  pinMode(dir1PinR, OUTPUT);
  pinMode(dir2PinR, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  stop_Stop();

  //加速度センサを使うため
  setupMMA8452();

  Serial.println("hello");
  /*
  //マイクロスイッチを使う
  pinMode(13, OUTPUT);
  pinMode(microswitch_pin, INPUT_PULLUP);
  */

  //測距センサ
  pinMode(A0, INPUT);
  pinMode(13, OUTPUT); //LED点灯用

}

void test() {
 //Serial.println(analogRead(A0));
//  set_angle(90);
//  go_Forward_Feedback(90,1000);
//  set_angle(90);
//  go_Back_Feedback(90,50);
//  set_angle(270);
//  go_Back_Feedback(270,1000);
set_angle(0);//270
//  go_Forward_Feedback(270,50);
  
//  go_Back_Feedback(90, 10000);
 //go_Forward_Feedback(-1, 1000);
  //drive_motors(200,200);
  //drive_motors(150,0);
//  drive_motors(0,100);
  //drive_motors(-200,-200);
  //delay(1000);
  //go_Back_Feedback(1,2000);
//for(int i = 0; i<3;i++){
//  drive_motors(155,-155);
//  delay(10);
//  set_angle(0);
//  go_Feedback(0, 1000, 250);
//  
//  drive_motors(155,-155);
//  delay(10);
//  set_angle(180);
//  go_Feedback(180, 1000, 250);
//}
}


void loop(){
  do_erase_all();
  //test();
}
