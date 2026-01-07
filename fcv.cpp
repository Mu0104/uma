#include <FastLED.h>
#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#define TIMER_INTERRUPT 1                   // Timer Interrupt Period

#define BOOST_CURRENT 50      
#define NORMAL_CURRENT 23.4  

#define I2C_SLAVE_ADDR 0x10
#define VESC_IDR 44
#define VESC_IDL 34

#define MOTOR_POLE_PAIRS 10 

#define DRIVER_ROLLER_PERIMETER 848     //mm  

#define THROTTLE_PIN 4
#define BOOST_PIN 3
#define DIRECTION_PIN1 1
#define DIRECTION_PIN2 2
#define BRAKE_PIN 42

#define BASE_VALUE 1350.0
#define MAX_VALUE 3400.0
#define BACK_MAX_VALUE 3450.0

#define MAX_SPEED 20.5 //km/h
#define BACK_MAX_SPEED 20.5 //km/h
#define BRAKE_CURRENT 20

//VESC
//------------------------------------------------------------------//
#define INPUT_DUTY 0
#define INPUT_CURRENT 1
#define INPUT_CURRENT_BRAKE	2
#define INPUT_RPM	3
#define INPUT_POS 4
#define INPUT_HAND_BRAKE 12

TaskHandle_t task_handl;

////LED
#define PIN_LED    21 
#define NUM_LEDS   1 
CRGB leds[NUM_LEDS];

// CAN
#define SPI_CLOCK_PIN 12
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 13

// WiFi
char ssid1[] = "siamesecat";
char pass1[] = "06290317";
char ssid2[] = "tgp";
char pass2[] = "tgp-wireless";
char ssid3[] = "Buffalo-G-4D60";
char pass3[] = "t84xcgg5ndnap";

bool second_flag = false;
bool third_flag = false;
bool wifi_flag = true;
unsigned char wifi_cnt = 0;
bool OTA_flag = false;

// CAN 
#define CAN_INT 7                   
MCP_CAN CAN(6);   
int TYPE2 = 1; 
int DLC2 = 4;
uint8_t buffers[4];
long unsigned int canId;
unsigned char len = 0;
byte datas[8];

uint8_t gear;
uint8_t boost_sw;
uint8_t brake_sw;
uint8_t boost_buff = 0;
uint8_t vesc_mode = 0;

float power;
int throttle;
int throttle_buff;
float set_speed;
float current_erpm_lim;
uint32_t erpm;
float erpm_buff;
float distance_trans = 50000 / 3 / DRIVER_ROLLER_PERIMETER * MOTOR_POLE_PAIRS;   //km/h -> erpm

class VESC_Message {
public:

  char duty_H;
  char duty_L;
  char current_H;
  char current_L;
  char rpm_3;
  char rpm_2;
  char rpm_1;
  char rpm_0;
  char amph_3;
  char amph_2;
  char amph_1;
  char amph_0;
  char amphc_3;
  char amphc_2;
  char amphc_1;
  char amphc_0;
  char fet_temp_H;
  char fet_temp_L;
  char motor_temp_H;
  char motor_temp_L;
  char total_current_H;
  char total_current_L;
  char pos_H;
  char pos_L;
  char odo_3;
  char odo_2;
  char odo_1;
  char odo_0;
  char voltage_H;
  char voltage_L;
};
VESC_Message VESC;

namespace std {
  constexpr bool signbit(float x); 
}

uint8_t VESC_Command;
int32_t VESC_Input;
int32_t odometer;
int32_t odometer_buff1;
int32_t odometer_buff2;
int32_t amp_hours1;
int32_t amph_hours_charged1;
int32_t amp_hours2;
int32_t amph_hours_charged2;
int32_t start_pos;

int duty_buff;
int current_buff;
int32_t rpm_buff;

float vesc_voltage1;
int fet_temp1;
int motor_temp1;
int total_current1;
int current_position1;
float vesc_voltage2;
int fet_temp2;
int motor_temp2;
int total_current2;
int current_position2;
int vesc_duty1;
float vesc_current1;
int32_t vesc_rpm1;
int vesc_duty2;
float vesc_current2;
int32_t vesc_rpm2;

float   velocity_1;
float   travel_1;
float   velocity_2;
float   travel_2;

uint16_t vel1;
uint16_t vel2;
uint16_t current1;
uint16_t current2;
uint16_t voltage1;


uint32_t i = 0;

// Timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
int iTimer8;
int iTimer10;
int iTimer50;

uint8_t OTA_mode = 0;
unsigned int pattern = 0;
//------------------------------------------------------------------//
//PROTOTYPE
void init_can();
void Recieve_can(void);
void sendData(int canId, int flameType, int DLC, byte datas[]);
void VESC_Send(int id,int32_t input);
void VESC_Set_Current(int set_current);
void IRAM_ATTR onTimer(void);
void timerInterrupt(void);
void taskDisplay(void *pvParameters);

void setup()
{
  init_can();
  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer); 

  xTaskCreatePinnedToCore(&taskDisplay, "taskDisplay", 4096, NULL, 1, &task_handl, 0);

  FastLED.addLeds<WS2812B, PIN_LED, GRB>(leds, NUM_LEDS);
  leds[0] = CRGB(0, 40, 60);
  FastLED.show();
  
  Wire.begin(8,9,400000U); 
  OTA_flag = false;
}

void loop()
{
  timerInterrupt(); 
  switch(pattern){
    case 0:
      break;

    case 10:
      WiFi.mode(WIFI_STA); 
      WiFi.begin(ssid1, pass1);  
      while (WiFi.status() != WL_CONNECTED) {
        wifi_cnt++;
        if( wifi_cnt > 20 ) {
          wifi_cnt = 0;
          second_flag = true;
          while(WiFi.status() == WL_CONNECTED ){
            WiFi.disconnect();
            delay(10);
          }
          break;
        }
        delay(200);  
      }
      if(second_flag) {
        wifi_cnt = 0;
        WiFi.mode(WIFI_STA); 
        WiFi.begin(ssid2, pass2); 
        while (WiFi.status() != WL_CONNECTED) {
          wifi_cnt++;
          if( wifi_cnt > 20 ) {
            wifi_cnt = 0;
            third_flag = true;
            while(WiFi.status() == WL_CONNECTED ){
              WiFi.disconnect();
              delay(10);
            }
            break;
          }
          delay(200);  
        }
      }
      if(third_flag) {
        WiFi.mode(WIFI_STA); 
        WiFi.begin(ssid3, pass3);  
        while (WiFi.status() != WL_CONNECTED) {
          wifi_cnt++;
          if( wifi_cnt > 20 ) {
            wifi_cnt = 0;
            wifi_flag = false;
            break;
          }
          delay(200);  
        }
      }
      OTA_flag = true;
      ArduinoOTA
      .setHostname("M5StampS3")
      .onStart([]() {})
      .onEnd([]() {})
      .onProgress([](unsigned int progress, unsigned int total) {})
      .onError([](ota_error_t error) {}); 
      ArduinoOTA.begin();
      leds[0] = CRGB(0, 0, 60);
      FastLED.show();
      pattern = 11;
      break;
    
    case 11:
      if(OTA_flag) ArduinoOTA.handle();
      break;
  }  
  
}

void taskDisplay(void *pvParameters){

  disableCore0WDT();

  while(1){  
    float aa = abs(velocity_1 * 100.0);
    if(velocity_1 < 0){
      vel1 = 65535 - (uint16_t)aa;
    }else{
      vel1 = (uint16_t)aa;
    }
    aa = abs(velocity_2 * 100.0);
    if(velocity_2 < 0){
      vel2 = 65535 - (uint16_t)aa;
    }else{
      vel2 = (uint16_t)aa;
    }
    aa = abs(vesc_current1 * 100.0);
    if(vesc_current1 < 0){
      current1 = 65535 - (uint16_t)aa;
    }else{
      current1 = (uint16_t)aa;
    }
    aa = abs(vesc_current2 * 100.0);
    if(vesc_current2 < 0){
      current2 = 65535 - (uint16_t)aa;
    }else{
      current2 = (uint16_t)aa;
    }
    aa = abs(vesc_voltage1 * 100.0);
    if(vesc_voltage1 < 0){
      voltage1 = 65535 - (uint16_t)aa;
    }else{
      voltage1 = (uint16_t)aa;
    }
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write((uint8_t)(vel1 & 0xff));
    Wire.write((uint8_t)(vel1 >> 8 & 0xff));
    Wire.write((uint8_t)(vel2 & 0xff));
    Wire.write((uint8_t)(vel2 >> 8 & 0xff));
    Wire.write((uint8_t)(current1 & 0xff));
    Wire.write((uint8_t)(current1 >> 8 & 0xff));
    Wire.write((uint8_t)(current2 & 0xff));
    Wire.write((uint8_t)(current2 >> 8 & 0xff));
    Wire.write((uint8_t)(voltage1 & 0xff));
    Wire.write((uint8_t)(voltage1 >> 8 & 0xff));
    Wire.write(gear);
    Wire.write(boost_sw);
    Wire.write(OTA_flag);
    Wire.endTransmission(true);
    delay(50);
    Wire.requestFrom(I2C_SLAVE_ADDR, 1);
    while(Wire.available())
      OTA_mode = Wire.read();
    if(OTA_mode == 10 && !pattern){
      pattern = 10;
      leds[0] = CRGB(0, 60, 20);
      FastLED.show();
    }
    if(!OTA_mode){
      leds[0] = CRGB(0, 40, 60);
      FastLED.show();
      pattern = 0;
    }
  }
}


void timerInterrupt(void) {

  if (interruptCounter > 0) {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);

    Recieve_can();
    
    iTimer10++;
    //10ms timerInterrupt
    switch(iTimer10){
    case 1:       
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      break;
    case 5:
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
    case 10:
      iTimer10 = 0;
      break;
    }

    iTimer50++;
    //50ms timerInterrupt
    switch (iTimer50) {
    case 10:
      if(!digitalRead(DIRECTION_PIN1))
        gear = 1;
      else if(!digitalRead(DIRECTION_PIN2))
        gear = 2;
      else 
        gear = 0;
      break;

    case 20:
      throttle = analogRead(THROTTLE_PIN);
      throttle_buff = throttle - BASE_VALUE;
        power = (float)throttle_buff / (MAX_VALUE - BASE_VALUE);
        if(throttle > MAX_VALUE)
        power = 1.0;

      if(throttle < BASE_VALUE)
        power = 0.0;
      
      if(gear == 1)
        set_speed = MAX_SPEED * power;
      else if(gear == 2)
        set_speed = -1.0 * BACK_MAX_SPEED * power;
      else 
        set_speed = 0;
      
      current_erpm_lim = set_speed * distance_trans;
      erpm_buff = abs(current_erpm_lim);
      if(current_erpm_lim < 0){
        erpm = 0xffffffff - (uint32_t)erpm_buff;
      }else{
        erpm = (uint32_t)erpm_buff;
      }
      break;

    case 30:
      boost_sw = digitalRead(BOOST_PIN);
      break;
    
    case 40: 
      brake_sw = digitalRead(BRAKE_PIN);
      break;

    case 50:
      if(boost_sw != boost_buff){
        boost_buff = boost_sw;
        if(boost_sw)
          VESC_Set_Current(BOOST_CURRENT);
        else
          VESC_Set_Current(NORMAL_CURRENT);
      }else{
        switch(vesc_mode){
          case 0:
            VESC_Send(INPUT_RPM,erpm);
            break;

          case 1:
            VESC_Send(INPUT_CURRENT_BRAKE,BRAKE_CURRENT);
            break;

          case 2:
            VESC_Send(INPUT_HAND_BRAKE,BRAKE_CURRENT);
            break;
        }
      }
      iTimer50 = 0;
      break;
    }
  }
}

void init_can()
{
  SPI.begin(SPI_CLOCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  CAN.begin(SPI,MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);
}
void VESC_Set_Current(int set_current){
  uint8_t buffer[8];
  int set_current_buff = set_current * 1000;
  int set_current_min = -1.0 * set_current * 1000;
      set_current_min = 0xFFFFFFFF + set_current_min;

  buffer[0] = (set_current_min >> 24) & 0xFF;
  buffer[1] = (set_current_min >> 16) & 0xFF;
  buffer[2] = (set_current_min  >> 8  )  & 0xFF;
  buffer[3] = set_current_min & 0xFF;
  buffer[4] = (set_current_buff >> 24) & 0xFF;
  buffer[5] = (set_current_buff >> 16) & 0xFF;
  buffer[6] = (set_current_buff  >> 8  )  & 0xFF;
  buffer[7] = set_current_buff & 0xFF;
  sendData(21 << 8 | VESC_IDR, TYPE2, 8, buffer);
  sendData(21 << 8 | VESC_IDL, TYPE2, 8, buffer);
}
//------------------------------------------------------------------//
void VESC_Send(int id,int32_t input){
  uint32_t input_data;
  int32_t set_data = input;
  uint8_t buffer[8];
  int total_idR,total_idL;

  total_idR = id << 8 | VESC_IDR;
  total_idL = id << 8 | VESC_IDL;

  switch(id){
    case 0:
      if(set_data < 0){
        input_data = 0xFFFFFFFF + (set_data * 1000);
      }else{
        input_data = set_data * 1000;
      }
      buffer[0] = (input_data >> 24) & 0xFF;
      buffer[1] = (input_data >> 16) & 0xFF;
      buffer[2] = (input_data  >> 8  )  & 0xFF;
      buffer[3] = input_data & 0xFF;
      sendData(total_idR, TYPE2, DLC2, buffer);
      sendData(total_idL, TYPE2, DLC2, buffer);
    break;

    case 1:
      if(set_data < 0){
        input_data = 0xFFFFFFFF + (set_data * 1000);
      }else{
        input_data = set_data * 1000;
      }
      buffer[0] = (input_data >> 24) & 0xFF;
      buffer[1] = (input_data >> 16) & 0xFF;
      buffer[2] = (input_data  >> 8  )  & 0xFF;
      buffer[3] = input_data & 0xFF;
      sendData(total_idR, TYPE2, DLC2, buffer);
      sendData(total_idL, TYPE2, DLC2, buffer);
    break;

    case 2:
      if(set_data < 0){
        input_data = 0xFFFFFFFF + (set_data * 1000);
      }else{
        input_data = set_data * 1000;
      }
      buffer[0] = (input_data >> 24) & 0xFF;
      buffer[1] = (input_data >> 16) & 0xFF;
      buffer[2] = (input_data  >> 8  )  & 0xFF;
      buffer[3] = input_data & 0xFF;
      sendData(total_idR, TYPE2, DLC2, buffer);
      sendData(total_idL, TYPE2, DLC2, buffer);
    break;

    case 3:
      set_data = set_data;
      if(set_data < 0){
        input_data = 0xFFFFFFFF + set_data;
      }else{
        input_data = set_data;
      }
      buffer[0] = (input_data >> 24) & 0xFF;
      buffer[1] = (input_data >> 16) & 0xFF;
      buffer[2] = (input_data  >> 8  ) & 0xFF;
      buffer[3] = input_data & 0xFF;
      sendData(total_idR, TYPE2, DLC2, buffer);
      sendData(total_idL, TYPE2, DLC2, buffer);
      break;
    
    case 4:
      input_data = set_data * 1000000;
      buffer[0] = (input_data >> 24) & 0xFF;
      buffer[1] = (input_data >> 16) & 0xFF;
      buffer[2] = (input_data  >> 8  ) & 0xFF;
      buffer[3] = input_data & 0xFF;
      sendData(total_idR, TYPE2, DLC2, buffer);
      sendData(total_idL, TYPE2, DLC2, buffer);
      break;
    
    case 12:
      if(set_data < 0){
        input_data = 0xFFFFFFFF + (set_data * 1000);
      }else{
        input_data = set_data * 1000;
      }
      buffer[0] = (input_data >> 24) & 0xFF;
      buffer[1] = (input_data >> 16) & 0xFF;
      buffer[2] = (input_data  >> 8  )  & 0xFF;
      buffer[3] = input_data & 0xFF;
      sendData(total_idR, TYPE2, DLC2, buffer);
      sendData(total_idL, TYPE2, DLC2, buffer);
    break;
    
  }
}
//CAN Send
//------------------------------------------------------------------//
void sendData(int canId, int flameType, int DLC, byte datas[]){
  CAN.sendMsgBuf(canId, flameType, DLC, datas);
  String data = "None";
}

void Recieve_can(){
  String data = "None";
  if(!digitalRead(CAN_INT))
  {
    CAN.readMsgBuf(&canId, &len, datas);

    if(canId==(VESC_IDR| 0x80000900) || canId==(VESC_IDL| 0x80000900)){
      for(i = 0; i < 8; i++){
        switch (i) {
          case 0:
            VESC.rpm_3 = (char)datas[i];
            break;
          case 1:
            VESC.rpm_2 = (char)datas[i];
            break;
          case 2:
            VESC.rpm_1 = (char)datas[i];
            break;
          case 3:
            VESC.rpm_0 = (char)datas[i];
            break; 
          case 4:
            VESC.current_H = (char)datas[i];
            break;          
          case 5:
            VESC.current_L = (char)datas[i];
            break;
          case 6:
            VESC.duty_H = (char)datas[i];
            break;
          case 7:
            VESC.duty_L = (char)datas[i];
            break;       
        }
      }  
        
      duty_buff = ((VESC.duty_H << 8 & 0xFF00) | (VESC.duty_L & 0xFF));
      current_buff = ((VESC.current_H << 8 & 0xFF00) | (VESC.current_L & 0xFF));
      rpm_buff = ((VESC.rpm_3 << 24 & 0xFF000000) | (VESC.rpm_2 << 16 & 0x00FF0000) | (VESC.rpm_1 << 8 & 0x0000FF00) | (VESC.rpm_0 & 0xFF));
        
      if(canId==(VESC_IDR| 0x80000900)){
        if(duty_buff >= 32768) {
          vesc_duty1 = (duty_buff - 65535) ;
        } else {
          vesc_duty1 = duty_buff ;          
        }
        if(current_buff >= 32768) {
          vesc_current1 = (float)(current_buff - 65535) / 10.0;
        } else {
          vesc_current1 = (float)current_buff / 10.0;          
        }
        if(rpm_buff >= 0x80000000) {
          vesc_rpm1 = (rpm_buff - 0xFFFFFFFF);
        } else {
          vesc_rpm1 = rpm_buff;          
        }
        velocity_1 = (float)vesc_rpm1 / distance_trans;
      }else{
        if(duty_buff >= 32768) {
          vesc_duty2 = (duty_buff - 65535) ;
        } else {
          vesc_duty2 = duty_buff ;          
        }
        if(current_buff >= 32768) {
          vesc_current2 = (float)(current_buff - 65535) / 10.0;
        } else {
          vesc_current2 = (float)current_buff / 10.0;          
        }
        if(rpm_buff >= 0x80000000) {
          vesc_rpm2 = (rpm_buff - 0xFFFFFFFF);
        } else {
          vesc_rpm2 = rpm_buff;          
        }
        velocity_2 = (float)vesc_rpm2 / distance_trans;
      }
    }
      

    if(canId==(VESC_IDR| 0x80001b00) || canId==(VESC_IDL| 0x80001b00)){
      for(i = 0; i < 6; i++){
        switch (i) {
          case 0:
            VESC.odo_3 = (char)datas[i];
            break;
          case 1:
            VESC.odo_2= (char)datas[i];
            break;
          case 2:
            VESC.odo_1 = (char)datas[i];
            break;
          case 3:
            VESC.odo_0 = (char)datas[i];
            break; 
          case 4:
            VESC.voltage_H = (char)datas[i];
            break;          
          case 5:
            VESC.voltage_L = (char)datas[i];
            break;    

        }
      }  
      if(canId==(VESC_IDR| 0x80001b00)){
        odometer_buff1 = ((VESC.odo_3 << 24 & 0xFF000000) | (VESC.odo_2 << 16 & 0x00FF0000) | (VESC.odo_1 << 8 & 0x0000FF00) | (VESC.odo_0 & 0xFF));
        travel_1 = (float)(odometer_buff1 - start_pos) / (MOTOR_POLE_PAIRS * 2.0 * 3.0) * DRIVER_ROLLER_PERIMETER / 1000.0;
        vesc_voltage1 = (float)((VESC.voltage_H << 8 & 0xFF00) | (VESC.voltage_L & 0xFF)) / 10.0;
      }else{
        odometer_buff2 = ((VESC.odo_3 << 24 & 0xFF000000) | (VESC.odo_2 << 16 & 0x00FF0000) | (VESC.odo_1 << 8 & 0x0000FF00) | (VESC.odo_0 & 0xFF));
        travel_2 = (float)(odometer_buff2 - start_pos) / (MOTOR_POLE_PAIRS * 2.0 * 3.0) * DRIVER_ROLLER_PERIMETER / 1000.0;
      }    
    }
  }
}
// IRAM
//------------------------------------------------------------------//
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  portEXIT_CRITICAL_ISR(&timerMux);
}