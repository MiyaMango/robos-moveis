#include <esp_now.h>
#include <WiFi.h>

//potentiometer stuff
int pot1pin = 32;
int pot1value = 0;
int old_pot1value = 0;

int pot2pin = 33;
int pot2value = 0;
int old_pot2value = 0;

void pot_to_speedndir(int value, int* speed, int*dir){
  const int mid = 2048;
  //get dir
  *dir = (value < mid) ? 0 : 1;

  //get speed
  if (value < mid) {
        *speed = map(value, 0, mid, 255, 160);
    } else {
        *speed = map(value, mid, 4095, 160, 255);
    }

}

// RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x88, 0x57, 0x21, 0xB1, 0xEF, 0xE0};

// Struct to send data
// Must match the receiver structure
typedef struct struct_message {
    int speed_lefttrack;
    int speed_righttrack;
    int dir_lefttrack; // 1 = forward 0 = reverse
    int dir_righttrack; // 1 = forward 0 = reverse
    int chksum;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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

  //register our OnDataSent function to be the callback when data is sent
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  // Zero out the peerInfo structure to ensure no garbage data causes issues
  memset(&peerInfo, 0, sizeof(peerInfo));
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  pot1value = analogRead(pot1pin);
  pot_to_speedndir(pot1value, &myData.speed_lefttrack, &myData.dir_lefttrack);
  
  pot2value = analogRead(pot2pin);
  pot_to_speedndir(pot2value, &myData.speed_righttrack, &myData.dir_righttrack);
  
  // Send message via ESP-NOW if the value is not the same as before
  if(old_pot1value != pot1value || old_pot2value != pot2value){
    myData.chksum = myData.speed_lefttrack + myData.speed_righttrack + myData.dir_lefttrack + myData.dir_righttrack;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }

  old_pot1value = pot1value;
  old_pot2value = pot2value;

  delay(50);
}