#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Stepper.h>
#include <stdint.h>

/*
@autor yoshida keisuke
y軸アームと水平他関節アーム制御用プログラム
*/

/*pram_setting*/
//y_motor_pram
const long stepsPerRevolution = 1000;       //モータの1回転あたりのステップ数
const long ballScrewPitch = 5;        // ねじ軸の1回転に伴い、ナットが軸方向に進む距離[mm]
long y_pos_max = 320;  
long y_pos_min = 50;
long target_y_pos;
long pre_pos_y = 0;
long step_y ;

//mult_link_arm
int Pulse_per_Rev = 1600.0;               //
float gear_ratio[3] = {20.25,20.25,4.0} ; //各モータのギア比

//encoder param
const int encoderResolution = 1000;       //encoder分解能
uint16_t zero_pos_enc[3] = {535,610,765}; //[x,y] = [0,0]の際のエンコーダ値
int pre_angle[3] = {0,0,0};               //n-1の絶対角度(ラジアン) ここがstep=0の原点になっている。
int target_angle[3] = {0,0,0};

/*pin_setup*/
//y_slider
int cwPlusPin = 10;   // CW + ピン
int cwMinusPin = 11;  // CW - ピン
int ccwPlusPin = 12;  // CCW + ピン
int ccwMinusPin = 13; // CCW - ピン
Stepper Y_StepperCW(stepsPerRevolution,cwPlusPin,cwMinusPin);    //時計まわり
Stepper Y_StepperCCW(stepsPerRevolution,ccwPlusPin,ccwMinusPin); //半時計まわり

//mult_link_arm
AccelStepper stepper1(AccelStepper::DRIVER, 3, 4);
AccelStepper stepper2(AccelStepper::DRIVER, 5, 6);
AccelStepper stepper3(AccelStepper::DRIVER, 7, 8);
MultiStepper steppers;

/*serial_param*/
char header = 'S';
char footer = 'E';
String receivedData = ""; 
boolean receivingData = false;



void setup() {
  Serial.begin(115200);
  //y_arm_setup
  pinMode(cwPlusPin, OUTPUT);
  pinMode(cwMinusPin, OUTPUT);
  pinMode(ccwPlusPin, OUTPUT);
  pinMode(ccwMinusPin, OUTPUT);
  Y_StepperCW.setSpeed(500);
  Y_StepperCCW.setSpeed(500);
  //arm_setup
  stepper1.setMaxSpeed(1400);
  stepper2.setMaxSpeed(1400);
  stepper3.setMaxSpeed(1400);
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);
}

/*位置取得関数*/
//エンコーダから正確な現在位置を取得
void get_pos(){
  //get_encder_values
  int enc_val[3];
  enc_val[0] = analogRead(0);
  enc_val[1] = analogRead(1);
  enc_val[2] = analogRead(2);
  enc_val[0] &= 0x03FF;
  enc_val[1] &= 0x03FF;
  enc_val[2] &= 0x03FF;
  enc_val[0] = 0x03FF - enc_val[0];
  enc_val[1] = 0x03FF - enc_val[1];
  enc_val[2] = 0x03FF - enc_val[2];
  //encder_values → angle_values
  int diff_enc[3];
  diff_enc[0] = zero_pos_enc[0] - enc_val[0];
  diff_enc[1] = zero_pos_enc[1] - enc_val[1];
  diff_enc[2] = enc_val[2] - zero_pos_enc[2];
  for (int i = 0; i < 3; i++) {
    pre_angle[i] = (diff_enc[i] * 360.0 / encoderResolution) ;
  }
}

/*アーム位置キャリブレーション*/
void calb_arm_pos(){
  get_pos();
  stepper1.setCurrentPosition(pre_angle[0]);  
  stepper2.setCurrentPosition(pre_angle[1]);  
  stepper3.setCurrentPosition(pre_angle[2]); 
}

/*初期化関数*/
void Initialization() {
  //現在位置キャリブレーション
  calb_arm_pos();
  //ホームポジションへ移動
  move_home_pos();
}

/*ホームポジションへ移動*/
void move_home_pos(){
  calb_arm_pos();
  int home_y = 60;
  int home_angle[3] = {20,70,0};
  run_y_motor(home_y);
  run_multi_arm_motor(target_angle,step_num);
  calb_arm_pos();
}

// /*トマトを収穫ボックスへ運ぶ*/
// void spit_into_harvest_box(){
//   calb_arm_pos();
//   int home_y = 60;
//   int home_angle[3] = {20,80,90};
//   int step_num[3];
//   run_y_motor(home_y);
//   angle2step(home_angle,step_num);
//   run_multi_arm_motor(step_num[0],step_num[1],step_num[2]);    
//   calb_arm_pos();
// }

void move2target(){
  if (y_pos_min <= target_y_pos and target_y_pos <= y_pos_max){
      run_y_motor(target_y_pos);
      run_multi_arm_motor(target_angle,step_num);
      calb_arm_pos();
  }
}

void loop() {
 while (Serial.available() > 0) {
   char receivedChar = Serial.read();
   if (receivedChar == header) {
     receivedData = "";
     receivingData = true;
   } else if (receivedChar == footer && receivingData) {
    if (receivedData.equals("init")) {
      //初期化
      Initialization();
      String message = 'S'+String(pre_angle[0])+","+String(pre_angle[1])+","+String(pre_angle[2])+'E';
      Serial.println(message); 
    }else if(receivedData.equals("home")){
      //ホームポジションに移動
      move_home_pos();
      String message = 'S'+String(pre_angle[0])+","+String(pre_angle[1])+","+String(pre_angle[2])+'E';
      Serial.println(message);
    // }else if(receivedData.equals("spit")){
    //   //トマトを収穫ボックスへ運ぶ
    //   spit_into_harvest_box();
    //   String message = 'S'+String(pre_angle[0])+","+String(pre_angle[1])+","+String(pre_angle[2])+'E';
    //   Serial.println(message);
    }else {
      //指定位置に移動
      int _ = sscanf(receivedData.c_str(), "%d,%d,%d,%d", &target_y_pos, &target_angle[0], &target_angle[1], &target_angle[2]);
      move2target();
      String message = 'S'+String(pre_angle[0])+","+String(pre_angle[1])+","+String(pre_angle[2])+'E';
      Serial.println(message);
    }
    receivingData = false;
    receivedData = "";
   } else if (receivingData){
    receivedData += receivedChar;
   }
 }
}

void run_y_motor(int target_pos){
  /*y_movement2step*/
  //目標位置(mm)⇨移動距離→ステップ数変換(y軸)
  //目標位置⇨移動距離
  long move_distance = (target_pos - pre_pos_y);
  pre_pos_y = target_pos;
  //移動距離→ステップ数変換
  long move_y_steps =  move_distance / ballScrewPitch * stepsPerRevolution ;
  if (move_y_steps > 0){
    while(move_y_steps - 32767 > 0){
        move_y_steps = move_y_steps - 32767;
        Y_StepperCCW.step(32767);
      }
    Y_StepperCCW.step(move_y_steps);
  }else{
    move_y_steps = abs(move_y_steps);
    while(move_y_steps - 32767 > 0){
        move_y_steps = move_y_steps - 32767;
        Y_StepperCW.step(32767);
      }
    Y_StepperCW.step(move_y_steps);          
  }
  return move_y_steps;
}

/*モータ動作関数*/
void run_multi_arm_motor(int *target_angle,int *step_num){
  long positions[3];
  for (int i = 0; i < 3; i++) {
    int diff_angle = target_angle[i] - pre_angle[i] ;
    positions[i] = diff_angle / 360.0 * Pulse_per_Rev * gear_ratio[i] ;
  }
  positions[0] = step1;
  positions[1] = step2;
  positions[2] = step3;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
}
