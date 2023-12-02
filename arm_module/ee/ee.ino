#include <MsTimer2.h>

#define CUTTER_PWM 5                
#define CUTTER_A 3                                    
#define CUTTER_B 4                  
#define TSW 6                       // Close_cutter_SW
#define BSW 7                       // Open_cutter_SW
#define OPTICAL_SENSOR 1                   // 光センサ
#define EDF_PWM 9



/*光センサ*/
int sample_num = 10;
int sensor_threshold = 900;           // 光センサのしきい値の初期化

/*吸引機構*/
int InitEDF_PWM = 100;                // EDF起動時のPWM
int suction_pwm = 200;                // EDF吸引時のPWM(serialで受信)

/*顎*/
int cutter_pwm = 220;
int CloseTime = 8000; //fingerセンサしまる上限時間(ボタンを押せなかった時用)


/*serial_param*/
char header = 'S';
char footer = 'E';
String receivedData = ""; 
boolean receivingData = false;


/*
初期化関数
*/
void init_cutter(){
    finger_stop();
}
void init_fun(){
    suction_off();
}
void optical_sensor_setup(){
    //光センサは場所によってしきい値が変わるので初期のオフセット値設定
    int optical_sensor_cali[sample_num];
    for(int cnt = 0 ; cnt < sample_num ; cnt++){
        optical_sensor_cali[cnt] = analogRead(OPTICAL_SENSOR); //光センサの値取得
    }
    // 光センサ10のデータの平均値
    int optical_sensor_sum = 0; //10個の光センサデータの合計変数の初期化
    for(int cnt = 0 ; cnt < sample_num ; cnt++){
        optical_sensor_sum += optical_sensor_cali[cnt];  //10個の光センサデータの合計
    }
    double optical_sensor_mean = (double)optical_sensor_sum / (double)(sample_num); // 光センサの平均値
    double t = 0; //偏差変数の初期化
    for(int cnt = 0 ; cnt < sample_num ; cnt++){
        t += (double)optical_sensor_cali[cnt] - (double)optical_sensor_mean; //偏差
    }
    double optical_sensor_std = sqrt(t / (double)(sample_num)); //標準偏差
    
    /*センサのしきい値の設定*/
    if(optical_sensor_std > 100){ 
        sensor_threshold = 900; //分散が大きすぎる場合は900
    }else{ 
        sensor_threshold = (int)optical_sensor_mean + 120; //平均値+120をしきい値とする
    }
}
void setup() {
  //pin_setup
  pinMode(CUTTER_A, OUTPUT);         
  pinMode(CUTTER_B, OUTPUT);         
  pinMode(CUTTER_PWM, OUTPUT);       
  pinMode(TSW, INPUT_PULLUP);       
  pinMode(BSW, INPUT_PULLUP);       
  pinMode(OPTICAL_SENSOR, INPUT);          
  pinMode(EDF_PWM, OUTPUT);             
  Serial.begin(115200);
  
  //motor and optical_sensor setup 
  init_cutter();
  init_fun();
  optical_sensor_setup();
}

/*基本動作*/
/*---------------------------------------------------------------*/
/*顎モータ制御*/
void finger_open(){
    digitalWrite(CUTTER_A, HIGH);  
    digitalWrite(CUTTER_B, LOW);
    while(digitalRead(BSW)!=0){
        analogWrite(CUTTER_PWM, cutter_pwm);
    }
    finger_stop();
}
void finger_close(){
    digitalWrite(CUTTER_A, LOW);
    digitalWrite(CUTTER_B, HIGH);
    long startTime = millis();
    bool time_out_flag = false;
    delay(100);    
    while(digitalRead(TSW)!=0 or time_out_flag){  
        long currentTime = millis();
        long past_time = currentTime- startTime;
        if(past_time>=7000){
          time_out_flag = true  ;
        }
        analogWrite(CUTTER_PWM, cutter_pwm);
    }
    finger_stop();
}
void finger_stop(){
    analogWrite(CUTTER_PWM, 0);
    digitalWrite(CUTTER_A, HIGH);
    digitalWrite(CUTTER_B, HIGH);
}

/*吸引機構*/
void suction_on(){
    analogWrite(EDF_PWM,suction_pwm);
}
void suction_off(){
    analogWrite(EDF_PWM,InitEDF_PWM);
}

/*光センサ確認*/
void Suction_judgement(){
    int optical_sensor = analogRead(OPTICAL_SENSOR);
    Serial.print(optical_sensor);Serial.print(",");Serial.println(sensor_threshold);
}
/*---------------------------------------------------------------*/


/*
口開ける
*/
void preparation(){
    suction_off();
    finger_open();
    Suction_judgement();
}

/*
トマト吸引する
*/
void suction_tomato(){
    suction_on();
    finger_stop();
    Suction_judgement();
}

/*
トマト噛み切る
*/
void cut_tomato_stem(){
    suction_on();
    finger_close();
    Suction_judgement();
}

/*
トマト吐き出す
*/
void spit_out(){
    suction_off();
    finger_open();
    Suction_judgement();
}


// void loop() {
//   if(Serial.available() > 0){
//     String input = Serial.readStringUntil(',');
//     int mode = input.toInt();
//     switch(mode){
//       case 0:
//         Serial.println("受け取りました");

//     }
//   }
// }


void loop() {
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == header) {
      receivedData = "";
      receivingData = true;
    } else if (receivedChar == footer && receivingData) {
      if (receivedData.equals("start")) {
        preparation();
        suction_tomato();
        cut_tomato_stem(); 
        preparation();
      }else {
        //すべてのモータ停止
        finger_stop();
        suction_off();
      }
      receivingData = false;
      receivedData = "";
    } else if (receivingData){
      receivedData += receivedChar;
    }
  }
}
