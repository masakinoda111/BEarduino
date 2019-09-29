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

/*調整するパラメータ --------------------------------------------------------------------------- */
#define WIDTH 1800          //黒板の横の長さ[mm]
#define HEIGHT 900          //黒板の縦の長さ[mm]
#define ERASER_LENGTH 125   //黒板ふきの長さ[mm]
#define D_CENTER 9          //重心のずれ
#define SPEED_H 0.18        //ロボットの水平に進む速さ[mm/ms]、HorizontalのH
#define SPEED_V 0.10        //ロボットの上に進む速さ[mm/ms]、VerticalのV
#define ACmeter_count 100   //過去何回分の加速度センサーの値の平均値を信用ある値として使うか
#define STOP_ANGLE_THRESHOLD 10    //set_angleでの許容幅
#define MOVE_ANGLE_THRESHOLD 0.1  //go_ForwardTurnでの許容幅（これを超えると進みながら曲がる）

//set_angleのパラメータ：
#define SET_ANGLE_SPEED 170 //回転速度の設定
#define E1 5 //5
#define E0 7   // 10          //移動時間の設定：keep(E1*abs(d_angle) + E0)

//go_Forward_Feedbackのパラメータ：
#define FORWARD_SPEED 255   //直進速度、3Vは75くらい
//#define C1 2
//#define C0 1                //左右のモータのspeed差の設定：C1*d_angle + C0
#define FORWARD_DELAY 5     //移動時間の設定。向いている方向が正しければ "3*FORWARD_DELAY" ms進む、ずれていれば "FORWARD_DELAY" ms進む

//go_Backward_Feedbackのパラメータ：
#define BACK_SPEED 150 //255
//#define D1 2
//#define D0 1
#define BACK_DELAY 3

// PID制御のパラメータ  e0 = d_angle; u = Kp*e0 + Ki*(e0+e1+e2) + Kd*(e0-e1);
#define Kp 4 //4.25
#define Ki 1  //1
#define Kd 0

float gForceX, gForceY, gForceZ; //↑を重力加速度の何倍か、に変換した値

float current_angle; //現在の機体が向いている方向。前はmoving_angleという名前だった。-180~180度。


//測距センサ
#define BUF_SIZE 8
#define SOKKYO_THRESHOLD 0.075
float voltage_buf[BUF_SIZE] = {};
int buf_counter = 0;
int buf_cnt = 0;
int buf_flag = 0;
float mean_of_voltage = 0.0;

float calculate_mean(){
  float sum = 0.0;
  for (int i = 0; i < BUF_SIZE; i++){
    sum += voltage_buf[i];
  }
  return sum/BUF_SIZE;
}

MMA8452Q accel;                   // create instance of the MMA8452 class

/***************motor control***************/
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

void go_Left(int speed_var)  //左に回転
{
  drive_motors(-speed_var,speed_var);
  //Serial.print("  GO LEFT\n");
}

void go_Right(int speed_var)  //右に回転
{
  drive_motors(speed_var,-speed_var);
  //Serial.print("  GO RIGHT\n");
}

void go_Back(int speed_var, int goal_angle)  //後ろに進む（注意！goal_angleは機体を向かせたい方向なので、後ろに進む場合進行方向とは真逆）
{
  drive_motors(-speed_var + D_CENTER*cos(goal_angle), -speed_var - D_CENTER*cos(goal_angle));
  //Serial.print("  GO BACK\n");
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

void software_reset() {
  asm volatile ("  jmp 0");
}

/***********加速度センサからの入力に応じた処理***********/

  /*ここに、gForceX, gForceY, gForceZの値に応じた処理を書く*/
  /*この加速度センサはx、y、z軸いずれも、鉛直上を向いているときに、1000を出力。*/
  /*姿勢は、車が鉛直壁に張り付いているとして、z軸方向が壁の法線方向と考えられるので、加速度のx軸、y軸成分(gForceX, gForeceY)を読み取り、それらの変化から姿勢を制御することを目標とする。*/

//現在の角度を測定

void getAngle(){  //読み取ったgの値から機体が向いている方向を計算して、current_angleに代入する
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
  current_angle= atan2(gForceY, gForceX)*180/Pi;

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
  //Serial.print("    current_angle = ");
  //Serial.print(current_angle);
  //Serial.print("  \n");
}

//current_angleとgoal_angleの差を求める。そのままだと差の絶対値が180を超えてしまう（正しい向きに回転しなくなるor激しく回転してしまうので困る）ので、差を-180～180の間に収める
float calculate_d_angle(float goal_angle){
  getAngle();
  float d_angle = current_angle - goal_angle;
  if(abs(d_angle)>180){
    if (d_angle>0){ d_angle-=360; }
    else { d_angle+=360; }
  }
  //Serial.print("goal_angle=");
  //Serial.println(goal_angle);
  //Serial.print("d_angle=");
  //Serial.println(d_angle);
  return d_angle;
}

//指定した角度に方向を直す。goal_angleが角度の目標値。goal_angleは-180度～180度の範囲で指定
void set_angle(float goal_angle){

  float d_angle = calculate_d_angle(goal_angle);

  while(abs(d_angle)> STOP_ANGLE_THRESHOLD){

    //目標角度に修正
    if(d_angle<0){
      go_Left(SET_ANGLE_SPEED); //回転スピードの設定
      keep(E1*abs(d_angle) + E0); //移動時間の設定
    }
    else{
      go_Right(SET_ANGLE_SPEED); //回転スピードの設定
      keep(E1*abs(d_angle) + E0); //移動時間の設定
    }
    stop_Stop();
    delay(100);

    d_angle = calculate_d_angle(goal_angle);
  }
  //Serial.print("SET ANGLE: ");
  //Serial.println(goal_angle);
  //Serial.print("\n");
}

int rightw_command(int e, int c_max)
{
  int c;
  c = e * (-1) + c_max;
  if(c > c_max){
    c = c_max;
  }
  if(c < 150){
    c = 150;
  }
  return c;
}

int leftw_command(int e, int c_max)
{
  int c;
  c = e + c_max;
  if(c > c_max){
    c = c_max;
  }
  if(c < 150){
    c = 150;
  }
  return c;
}

void go_Forward_Feedback(int goal_angle, int move_time) //前に進む、フィードバック制御あり。角度を常に一定にキープしながら進むことができる。move_timeは移動時間
{

  float e1=0; // 単位時間前の偏差
  float e2=0; // 2単位時間前の偏差
  float e0;
  int u=0;
  float outL, outR; //左右のモーターに出力される値
  float time_count=0;  //time_countは経過した時間。これがmove_timeを下回っている間、while loopが回り続ける

//  buf_counter = 0;  //ここ
//  buf_cnt = 0;
//  buf_flag = 0;
//  mean_of_voltage = 0.0;  //ここまで、必要なのかわからないけど、初期化を一応入れておく。

  while(time_count < move_time){
    float d_angle = calculate_d_angle(goal_angle);

      //測距センサ
      int tmp_val = 0;
      float tmp_voltage = 0.0;
      float sum_of_tmp_voltage = 0.0;
      float mean_of_tmp_voltage = 0.0;

      //Serial.print("buf_counter = ");
      //Serial.print(buf_counter);
      //Serial.print("   ");

      digitalWrite(13, LOW);
      for (int i = 0; i<500; i++){
        tmp_val = analogRead(A0);
        tmp_voltage = (tmp_val / 1024.0)*5;
        sum_of_tmp_voltage += tmp_voltage;
      }
      mean_of_tmp_voltage = sum_of_tmp_voltage / 500;
      voltage_buf[buf_counter] = mean_of_tmp_voltage;
      buf_counter++;
      if (buf_counter == BUF_SIZE){
        buf_counter = 0;
      }
      if (buf_cnt < BUF_SIZE){
        buf_cnt++;
      } 
      if (buf_cnt == BUF_SIZE){
        //Serial.print("電圧 mean_of_tmp_voltage: ");
        //Serial.print(mean_of_tmp_voltage);
        //Serial.print("   ");
        mean_of_voltage = calculate_mean();
        //Serial.print("mean_of_voltage = ");
        //Serial.println(mean_of_voltage);

        if (abs(mean_of_tmp_voltage - mean_of_voltage) > SOKKYO_THRESHOLD){
          //Serial.println("距離が変わりました");
          buf_cnt = 0;
          buf_flag = 0;
          digitalWrite(13, HIGH);  //シリアル通信しない時の確認用にLEDを点灯させる
          //delay(150);
          break;
        }

      }

    //以降、d_angleを0に収束させるフィードバック制御：
    if(abs(d_angle)>30){  //あまりに違う方向を向いている場合はその場で回転して角度を修正
      set_angle(goal_angle);
    }
    else  // 以下PID制御
    {
      e0 = d_angle;
      u = Kp*e0 + Ki*(e0+e1+e2) + Kd*(e0-e1);
      if(u>255){
        u = 255;
      }
      if(u<-255){
        u = -255;
      }
      outL = leftw_command(u, FORWARD_SPEED);
      outR = rightw_command(u, FORWARD_SPEED);     //右のタイヤが強めに働いてたので、見切りで減算しとく。
      drive_motors(outL,outR);

      e2 = e1;
      e1 = e0;

      //Serial.println("  GO FORWARD (PID)\n");
      delay(FORWARD_DELAY);
      time_count += FORWARD_DELAY ;
    }

    // else if(abs(d_angle) < MOVE_ANGLE_THRESHOLD)
    // {  //d_angleが許容幅（MOVE_ANGLE_THRESHOLD）以内なら、しばらくそのまままっすぐ進む
    //   go_Forward(FORWARD_SPEED, goal_angle);
    //   delay(3 * FORWARD_DELAY);
    //   time_count += 3 * FORWARD_DELAY ;
    // }
    // else if(d_angle > 0)  //d_angleが正なので、右に回転しながら前進。最初の二項はgo_Forwardと同じで、あとの二項がフィードバック部分
    // {
    //   outL = FORWARD_SPEED + D_CENTER*cos(goal_angle) + C1*d_angle + C0;  //C1、C0の値を調整してフィードバックの大きさを決める
    //   outR = FORWARD_SPEED - D_CENTER*cos(goal_angle) - C1*d_angle - C0;
    //   drive_motors(outL,outR);
    //   Serial.println("  GO FORWARD (FEEDBACK)\n");
    //   delay(FORWARD_DELAY);
    //   time_count += FORWARD_DELAY ;
    // }
    // else
    // {  //d_angleが負であることに注意。
    //   outL = FORWARD_SPEED + D_CENTER*cos(goal_angle) + C1*d_angle - C0;
    //   outR = FORWARD_SPEED - D_CENTER*cos(goal_angle) - C1*d_angle + C0;
    //   drive_motors(outL,outR);
    //   Serial.println("  GO FORWARD (FEEDBACK)\n");
    //   delay(FORWARD_DELAY);
    //   time_count += FORWARD_DELAY ;
    // }
  }
}

void go_Back_Feedback(int goal_angle, int move_time) //後ろに進む、フィードバック制御あり。角度を常に一定にキープしながら進むことができる。move_timeは移動時間
{
  float e1=0; // 単位時間前の偏差
  float e2=0; // 2単位時間前の偏差
  float e0;
  int u=0;
  float outL, outR; //左右のモーターに出力される値
  float time_count=0;  //time_countは経過した時間。これがmove_timeを下回っている間、while loopが回り続ける
  while(time_count < move_time){
    float d_angle = calculate_d_angle(goal_angle);

      //測距センサ
      int tmp_val = 0;
      float tmp_voltage = 0.0;
      float sum_of_tmp_voltage = 0.0;
      float mean_of_tmp_voltage = 0.0;

      //Serial.print("buf_counter = ");
      //Serial.print(buf_counter);
      //Serial.print("   ");

      digitalWrite(13, LOW);
      for (int i = 0; i<100; i++){
        tmp_val = analogRead(A2);
        tmp_voltage = (tmp_val / 1024.0)*5;
        sum_of_tmp_voltage += tmp_voltage;
      }
      mean_of_tmp_voltage = sum_of_tmp_voltage / 100.0;
      Serial.println(mean_of_tmp_voltage);
        if (mean_of_tmp_voltage < 4.6 || mean_of_tmp_voltage > 6){
          stop_Stop();
          //Serial.println("距離が変わりました");
          buf_cnt = 0;
          buf_flag = 0;
          digitalWrite(13, HIGH);  //シリアル通信しない時の確認用にLEDを点灯させる
          //delay(150);
          break;
        }
        
    //以降、d_angleを0に収束させるフィードバック制御：
    if(abs(d_angle)>45){  //あまりに違う方向を向いている場合はその場で回転して角度を修正
      set_angle(goal_angle);
    }
    else  // 以下PID制御
    {
      e0 = d_angle;
      u = Kp*e0 + Ki*(e0+e1+e2) + Kd*(e0-e1);
      /*if(u>255){
        u = 255;
      }
      if(u<-255){
        u = -255;
      }*/
      outL = -rightw_command(u, FORWARD_SPEED);
      outR = -leftw_command(u, FORWARD_SPEED);
      drive_motors(outL,outR);

      e2 = e1;
      e1 = e0;

      //Serial.println("  GO FORWARD (PID)\n");
      delay(FORWARD_DELAY);
      time_count += FORWARD_DELAY ;
    }


//    else if(abs(d_angle) < MOVE_ANGLE_THRESHOLD)
//    {  //d_angleが許容幅（MOVE_ANGLE_THRESHOLD）以内なら、しばらくそのまままっすぐ進む
//      go_Back(BACK_SPEED, goal_angle);
//      delay(3 * BACK_DELAY);
//      time_count += 3 * BACK_DELAY ;
//    }
//    else if(d_angle > 0)  //d_angleが正なので、右に回転しながら後退。最初の二項はgo_Backと同じで、あとの二項がフィードバック部分
//    {
//      outL = -BACK_SPEED + D_CENTER*cos(goal_angle) + D1*d_angle + D0;  //D1、D0の値を調整してフィードバックの大きさを決める
//      outR = -BACK_SPEED - D_CENTER*cos(goal_angle) - D1*d_angle - D0;
//      drive_motors(outL,outR);
//      Serial.println("  GO BACK (FEEDBACK)\n");
//      delay(BACK_DELAY);
//      time_count += BACK_DELAY ;
//    }
//    else
//    {  //d_angleが負であることに注意。
//      outL = -BACK_SPEED + D_CENTER*cos(goal_angle) + D1*d_angle - D0;  //D1、D0の値を調整してフィードバックの大きさを決める
//      outR = -BACK_SPEED - D_CENTER*cos(goal_angle) - D1*d_angle + D0;
//      drive_motors(outL,outR);
//      Serial.println("  GO BACK (FEEDBACK)\n");
//      delay(BACK_DELAY);
//      time_count += BACK_DELAY ;
//    }
  }
}

//消去アルゴリズム
void do_erase(){
  //set_angle(0);
  //go_Forward_Feedback(1.5, 20);
  //go_Back_Feedback(2,10);
  int i;
  for(i=0;i<2 /*0.5*HEIGHT/ERASER_LENGTH*/;i++){
    set_angle(0);
    delay(150);

//    go_Back_Feedback(0, 4); //ちょっと後ろに下がって消す

    go_Forward_Feedback(359,20000); //右にまっすぐ進む。int goal_angle, int move_time

    go_Right(150);  //右回転
    keep(850);
    delay(150);



    set_angle(180);
    delay(150);

//    go_Back_Feedback(180, 4); //ちょっと後ろに下がって消す

    go_Forward_Feedback(178,20000); //左にまっすぐ進む。

    go_Left(150); //左回転
    keep(850);
    delay(150);


  }
  go_Back_Feedback(179,10);
  set_angle(90);
//  set_angle(88);
  go_Forward_Feedback(89, 200);
  //set_angle(270);
//  go_Back_Feedback(270,175);
  //delay(HEIGHT/SPEED_V);
}


//消去アルゴリズム
//ターンせずに消す
//上下方向
void do_erase_vertical(){
  go_Forward_Feedback(180, 20000);//0,20000
  go_Back_Feedback(0, 20);
  drive_motors(100,255);
  go_Back_Feedback(90,20000);
  delay(100);
  for(int i=0;i<7;i++){
    go_Forward_Feedback(90,20000); //上にまっすぐ進む。int goal_angle, int move_time

    //一列左にずれながら下に消す
    drive_motors(-70,-255);
    delay(500);
    drive_motors(-255,-70);
    delay(400);
    go_Back_Feedback(90,20000);  //140
    
  }
  go_Forward_Feedback(90,2000);
  //go_Back_Feedback(90,10);
  drive_motors(255,150);
  
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
//  set_angle(270);
//  go_Forward_Feedback(270,50);
  
    go_Back_Feedback(270, 10000);
    drive_motors(255,0);
    delay(200);
    go_Back_Feedback(90, 10000);
    drive_motors(150,255);
    delay(300);
//  go_Forward_Feedback(90, 10000);
  //drive_motors(200,200);
  //drive_motors(150,0);
//  drive_motors(0,100);
  //drive_motors(-200,-200);
  //delay(1000);
}


void loop(){
  //do_erase();
  //do_erase_parallel();
  //do_erase_vertical();
  //set_angle(0);
  //go_Back_Feedback(0, 100);
  test();
//  go_Forward_Feedback(0, 1000);  //デモ動画用
//  go_Back_Feedback(0, 450);
  //getAngle();
}
