#include <esp_now.h>
#include <WiFi.h>

// Motor A pinout
const int motor1Pin1 = 26;
const int motor1Pin2 = 27; 
const int enable1Pin = 25;

// Motor B pinout 
const int motor2Pin1 = 18;
const int motor2Pin2 = 19; 
const int enable2Pin = 21;

// PWM properties
const int freq = 20000;
const int pwmCh1 = 0;
const int pwmCh2 = 1;
const int resolution = 8;

// connection stuff
int time_since_last_signal = 0;

// struct to receive data
typedef struct struct_message {
    int speed_lefttrack;
    int speed_righttrack;
    int dir_lefttrack; // 1 = forward 0 = reverse
    int dir_righttrack; // 1 = forward 0 = reverse
    int chksum;
} struct_message;

struct_message myData;

// function to verify chksum
bool check_sum(struct_message s){
  int sum = s.speed_lefttrack + s.speed_righttrack + s.dir_lefttrack + s.dir_righttrack;
  if(sum == s.chksum) return true;
  else return false;
}

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info * info, const uint8_t *incomingData, int len) {
  const struct_message *inc = (const struct_message *)incomingData;
  if(!check_sum(*inc)) return;

  memcpy(&myData, incomingData, sizeof(myData));
  time_since_last_signal = 0;
}

//motor control functions
void process_motors(){

  //left motor first
  if(myData.dir_lefttrack == 1){
    //go forward
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  }else{
    //go backward
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  }

  //write speed in leftmotor
  ledcWrite(enable1Pin, myData.speed_lefttrack);

  // ------------ //

  //then right motor
  if(myData.dir_righttrack == 1){
  //go forward
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  }else{
  //go backward
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }

  //write speed in rightmotor
  ledcWrite(enable2Pin, myData.speed_righttrack);
}

void kill_motors(){
  ledcWrite(enable1Pin, 0);
  ledcWrite(enable2Pin, 0);
}
 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  
  //set device as wifi station
  WiFi.mode(WIFI_STA);

  //init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  //register our OnDataRecv function to be the callback function triggered on data receive
  esp_now_register_recv_cb(OnDataRecv);

  //setup for motors
  myData.speed_lefttrack = 0;
  myData.speed_righttrack = 0;

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  ledcAttachChannel(enable1Pin, freq, resolution, pwmCh1);
  ledcAttachChannel(enable2Pin, freq, resolution, pwmCh2);
}

void loop() {
  //if(time_since_last_signal++ < 20) process_motors();
  //else kill_motors();
  process_motors();
  delay(50);
}