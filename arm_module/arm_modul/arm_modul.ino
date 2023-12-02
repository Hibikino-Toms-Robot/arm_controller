#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Stepper.h>
#include <stdint.h>

/*
@autor yoshida keisuke
z軸アームと水平他関節アーム制御用プログラム
*/

/*pram_setting*/
//z_motor_pram
const int stepsPerRevolution = 800;       //モータの1回転あたりのステップ数
const float ballScrewPitch = 10.0;        // ねじ軸の1回転に伴い、ナットが軸方向に進む距離[mm]
float z_pos_max = 750.0;
float z_pos_min = 100.0;
int target_z_pos;
float pre_pos_z = 0;


//mult_link_arm
int Pulse_per_Rev = 1600.0;               //
float gear_ratio[3] = {20.25,20.25,4.0} ; //各モータのギア比

//encoder param
const int encoderResolution = 1000;       //encoder分解能
uint16_t zero_pos_enc[3] = {540,610,645}; //[x,y] = [0,0]の際のエンコーダ値
int pre_angle[3] = {0,0,0};               //n-1の絶対角度(ラジアン) ここがstep=0の原点になっている。
int target_angle[3] = {0,0,0};

/*pin_setup*/
//z_slider
int cwPlusPin = 10;   // CW + ピン
int cwMinusPin = 11;  // CW - ピン
int ccwPlusPin = 12;  // CCW + ピン
int ccwMinusPin = 13; // CCW - ピン
Stepper Z_StepperCW(stepsPerRevolution,cwPlusPin,cwMinusPin);    //時計まわり
Stepper Z_StepperCCW(stepsPerRevolution,ccwPlusPin,ccwMinusPin); //半時計まわり

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
  //z_arm_setup
  pinMode(cwPlusPin, OUTPUT);
  pinMode(cwMinusPin, OUTPUT);
  pinMode(ccwPlusPin, OUTPUT);
  pinMode(ccwMinusPin, OUTPUT);
  Z_StepperCW.setSpeed(500);
  Z_StepperCCW.setSpeed(500);
  //arm_setup
  stepper1.setMaxSpeed(800);
  stepper2.setMaxSpeed(800);
  stepper3.setMaxSpeed(800);
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
  Serial.println("初期化完了");
}


/*ホームポジションへ移動*/
void move_home_pos(){
  int home_z = 100;
  int home_angle[3] = {20,70,0};
  int step_num[3];
  angle2step(home_angle,step_num);
  int step_z = z_movement2step(home_z);
  run_motor(step_z,step_num[0],step_num[1],step_num[2]);
  calb_arm_pos();
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
      }else if(receivedData.equals("home")){
        //ホームポジションに移動
        move_home_pos();
      }else {
        //指定位置に移動
        int _ = sscanf(receivedData.c_str(), "%d,%d,%d,%d", &target_z_pos, &target_angle[0], &target_angle[1], &target_angle[2]);
        int step_z = z_movement2step(target_z_pos);
        int step_num[3];
        angle2step(target_angle,step_num);
        run_motor(step_z,step_num[0],step_num[1],step_num[2]);
        calb_arm_pos();
        String message = 'S'+String(pre_angle[0])+","+String(pre_angle[1])+","+String(pre_angle[2])+'M';
        Serial.println(message);
      }
      receivingData = false;
      receivedData = "";
    } else if (receivingData){
      receivedData += receivedChar;
    }
  }
}



//目標位置(mm)⇨移動距離→ステップ数変換(z軸)
int z_movement2step(int target_pos){
  //目標位置⇨移動距離
  if (target_pos >= z_pos_max){
    target_pos = z_pos_max;
  }else if(target_pos <= z_pos_min){
    target_pos = z_pos_min;
  }
  int move_distance = target_pos - pre_pos_z;
  pre_pos_z = target_pos;
  //移動距離→ステップ数変換
  int move_steps = move_distance / ballScrewPitch * stepsPerRevolution ; 
  return move_steps;
}

//移動距離→ステップ数変換(水平他関節アーム)
void angle2step(int *target_angle,int *step_num){
  for (int i = 0; i < 3; i++) {
    int diff_angle = target_angle[i] - pre_angle[i] ;
    step_num[i] = diff_angle / 360.0 * Pulse_per_Rev * gear_ratio[i] ;
  }
}

/*モータ動作関数*/
void run_motor(int stepz, int step1, int step2, int step3){
  if (stepz > 0){
    Z_StepperCCW.step(abs(stepz));
  }else{
    Z_StepperCW.step(abs(stepz));            
  }
  long positions[3];
  positions[0] = step1;
  positions[1] = step2;
  positions[2] = -step3;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
}
