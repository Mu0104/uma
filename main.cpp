#include <SD.h>
#include <SPI.h>
#include <M5Unified.h>
#include <mcp_can.h>
#include <CircularBuffer.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <Wire.h>

//Define
//------------------------------------------------------------------//
#define TIMER_INTERRUPT 1     // Timer Interrupt Period

#define VSPI_CLOCK_PIN 18
#define VSPI_MOSI_PIN 23
#define VSPI_MISO_PIN 38

#define VESC_ID_F 34
#define VESC_ID_R 44
#define MOTOR_POLE_PAIRS 7.0

#define DRIVER_ROLLER_PERIMETER 283.00     //mm
#define IDLER_ROLLER_PERIMETER 62.8        //mm
#define IDLER_ROLLER_CON 1.06

#define CONTROL_FREQUENCY 100

#define MINIMUM_ERPM 700.0
#define SCALE 1000.0

#define BUFFERRecords 32 

#define ENC_PIN 1
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_SLAVE_ADDR 0x11
#define EMG_STAMP_ADDRESS 0x21

#define GEAR_RATIO 36.0
//#define SCREW_PITCH 1.0        //Screw 6mm
#define SCREW_PITCH 0.8        //Screw 5mm
#define UNINSTALL_DIST 6.0
#define HALL_RATE 5000
//#define HALL_RATE 149.02

#define EMG_PIN 36

//VESC
//------------------------------------------------------------------//
#define INPUT_DUTY 0
#define INPUT_CURRENT 1
#define INPUT_CURRENT_BRAKE	2
#define INPUT_RPM	3
#define INPUT_POS 4
#define INPUT_HAND_BRAKE 12
#define BRAKE_RELEASE 25

//ODrive State
//------------------------------------------------------------------//
#define IDLE 1              
#define STARTUP_SEQUENCE 2
#define FULL_CALIBRATION_SEQUENCE 3
#define MOTOR_CALIBRATION 4
#define ENCODER_INDEX_SEARCH 6
#define ENCODER_OFFSET_CALIBRATION 7
#define CLOSED_LOOP_CONTROL 8

//ODrive SET
//------------------------------------------------------------------//
#define INPUT_MODE_INACTIVE 0
#define INPUT_MODE_PASSTHROUGH 1
#define INPUT_MODE_VEL_RAMP	2
#define INPUT_MODE_POS_FILTER	3
#define INPUT_MODE_MIX_CHANNELS 4
#define INPUT_MODE_TRAP_TRAJ 5
#define INPUT_MODE_TORQUE_RAMP 6
#define INPUT_MODE_MIRROR 7
#define INPUT_MODE_Tuning 8
#define CONTROL_MODE_VOLTAGE_CONTROL 0
#define CONTROL_MODE_TORQUE_CONTROL 1
#define CONTROL_MODE_VELOCITY_CONTROL 2
#define CONTROL_MODE_POSITION_CONTROL 3

//ODrive Command
//------------------------------------------------------------------//
#define ODRIVE_INPUT_POS 0x0c
#define ODRIVE_INPUT_TORQUE 0x0e
#define ODRIVE_SET_VEL_LIMIT 0x0f

//ODrive Command STAMP
//------------------------------------------------------------------//
#define TORQUE_CONTROL 1
#define POSITION_CONTROL 2
#define POSITION_CONTROL_TRAJ 3
#define ODRIVE_CALIBRATION 4
#define ODRIVE_CLOSED_LOOP 5
#define ODRIVE_REBOOT 6
#define ODRIVE_IDLE 7
#define UP10 10
#define UP1 11
#define DOWN10 12
#define DOWN1 13
#define UNINSTALL 14
#define ODRIVE_OFFSET 15
#define CALIBRATION 16
#define ARM_CONTROL 17
#define EMG_STOP 18

//ODrive Error
//------------------------------------------------------------------//
#define ENCODER_SPI_FAIL 256   

//Stamp
#define STAMP_OTA 31
#define STAMP_OTA_END 32
#define INPUT_TRAJECTRY 0x20
#define INPUT_POSITION 0x31
#define INPUT_TORQUE 0x22
#define INPUT_OFFSET_DIST 0x23
#define SET_DATA1 0x13
#define SET_DATA2 0x14
#define SET_DATA3 0x15
#define SET_DATA4 0x16
#define STAMP_TOF_1 0x41
#define STAMP_TOF_2 0x42
#define STAMP_TOF_3 0x43
#define STAMP_TOF_4 0x45
#define STAMP_TOF_5 0x46
#define STAMP_TOF_6 0x47
#define STAMP_INIT_BARO 0x51


CircularBuffer<unsigned int, 10> emg_button_status_buffer;
CircularBuffer<unsigned int, 10> sens1_buffer;
CircularBuffer<unsigned int, 10> sens2_buffer;
CircularBuffer<unsigned int, 10> ODrive_vel_buffer;
CircularBuffer<unsigned int, 10> front_tof_status_buffer;
CircularBuffer<unsigned int, 10> rear_tof_status_buffer;
CircularBuffer<int, 20> encoder_status_buffer;

TaskHandle_t task_handl;

// Log
typedef struct {
  long          log_time;    
  unsigned char log_pattern;
  float         log_velocity_1;
  float         log_travel_1;
  int           log_current;
  float         log_voltage;

} RecordType;
unsigned int number_run = 1;

int write_data[6];
int read_data[6];
int read_datar[6];

volatile bool       finish_flag = false;
static RecordType   buffer1[2][BUFFERRecords];
static volatile int write_Bank = 0;
static volatile int buffer_Index[2] = {0, 0};


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

uint8_t dis1,dis2,dis3,dis4,dis5,dis6,dis7,dis8;
uint16_t distance1,distance2,distance3,distance4,distance5,distance6,distance7,distance8,distance9;
uint16_t avg_distance_F;
uint16_t avg_distance_R;

uint8_t VESC_Command;
int32_t VESC_Input;
int32_t odometer;
int32_t odometer_buff_F;
int32_t odometer_buff_R;
int32_t amp_hours;
int32_t amph_hours_charged;
int32_t start_pos_F;
int32_t start_pos_R;

int duty_buff;
int current_buff;
int32_t rpm_buff;

float vesc_voltage_F;
float vesc_voltage_R;
int fet_temp;
int motor_temp;
int total_current;
int current_position;
int vesc_duty;
int vesc_current_F;
int32_t vesc_rpm_F;
int32_t vesc_rpm_R;
int32_t set_erpm;
int32_t brake_current;
int32_t Low_brake_current;
int32_t hand_brake_current;

float   velocity_1;
float   travel_1;
float   velocity_2;
float   travel_2;
float   travel_2_buff;
float   velocity_3;
float   travel_3;
float   travel_3_buffer;
float   travel_3_buff;
float distance_trans = 50000 / 3 / DRIVER_ROLLER_PERIMETER * MOTOR_POLE_PAIRS;   //km/h -> erpm

float start_pos_Idree;

//trajectory_control
float set_speed;
float current_erpm_lim;
float target_pos;
float current_vel;
float current_pos;
float current_erpm;
float erpm_lim;
float deceleration_pos;
float Vo;
float stop_dist;
float dX;
unsigned int low_speed_dist;
float V_max;
float D_max;
float A_max;
unsigned int down_margin;
float accel_lim;
float decel_lim;
float vel_lim;
float s;
float Dmax_cor;
uint8_t current_limit;
float last_change_dist;

//position control
float pos_p;
float pos_i;
float pos_d;
unsigned int pos_p_ROM;
unsigned int pos_i_ROM;
unsigned int pos_d_ROM;
int position_control_erpm ;
int position_control_down_velocity;
int position_control_down_erpm ;

bool accel_flag = false;
bool cruise_flag = false;
bool decel_flag = false;
bool velocity_control_flag = false;
bool position_control_flag = false;
bool trajectory_control_flag = false;
bool current_brake_flag = false;
bool hand_brake_flag = false;
bool direction = false;

// Paramenters
unsigned int climb_height;
unsigned int climb_velocity;
unsigned int down_velocity;
unsigned int Manual_climb_velocity;
unsigned int Manual_Descend_velocity;
unsigned int position_control_velocity;
float climber_decel;
float climber_accel;
unsigned int climber_decel_ROM;
unsigned int climber_accel_ROM;
float down_decel;
float down_accel;
unsigned int down_accel_ROM;
unsigned int down_decel_ROM;
int starting_count;
int stop_wait;
unsigned int throttle_current;

//ODrive
float ODrive_Current_pos_F;
float ODrive_Current_pos_R;
uint32_t arm;
float arm_current_pos;
uint8_t Current_State_F;
uint8_t Current_State_R;
uint32_t ODrive_F_buff;
uint32_t ODrive_R_buff;
uint8_t ODrive_F[3];
uint8_t ODrive_R[3];
float torque_ODrive;
unsigned int torque_ODrive_ROM;
float ODrive_offset;
unsigned int ODrive_offset_ROM;
float ODrive_uninstall_distance;
unsigned int ODrive_uninstall_distance_ROM;
float ODrive_vbus;

typedef union {
  float val;
  byte ary[4];
  int i;
} Ta;
Ta a; 


uint8_t hallF_L,hallF_H,hallR_L,hallR_H;
uint16_t hall_F,hall_R;
uint16_t hall_target;
float ODrive_target_pos;
int ODrive_dX;

// CAN 
#define CAN1_INT 35   //VESC             
MCP_CAN CAN1(27);   
#define CAN2_INT 2    //ODrive             
MCP_CAN CAN2(19);   
#define CAN3_INT 32   //Stamp                   
MCP_CAN CAN3(33);   
int TYPE2 = 1; 
int DLC2 = 4;
uint8_t buffers[4];
long unsigned int canId;
long unsigned int canId_vesc;
long unsigned int canId_odrive;
unsigned char len = 0;
float trans;
uint32_t set_data;

int TYPE1 = 0; // Standard Format
int DLC1 = 8;
int Stamp_id = 255;
byte datas[8];
byte send_val[8];
byte send_data[8];
byte data_stamp[8];
uint8_t command_id;
uint8_t velF_H;
uint8_t velF_L;
uint8_t trv[3];
uint16_t velF_buff;
uint32_t trvF_buff;
uint8_t current_L;
uint8_t current_H;
float vesc_current_R;
uint16_t current_buff_F;

//I2C
uint8_t bytesReceived;
uint8_t vel_L;
uint8_t vel_H;
uint8_t trv_1;
uint8_t trv_2;
uint8_t trv_3;
uint8_t alt_1;
uint8_t alt_2;
uint8_t alt_3;
uint16_t vel_buff;
uint32_t trv_buff;
uint32_t alt_buff;
uint32_t trv1;
uint32_t trv2;
uint16_t vel1;
uint16_t vel2;
uint16_t current_F;
uint16_t current_R;
uint32_t arm_buff;
uint8_t arm_1,arm_2;
float arm_height;
float arm_start_pos;
float arm_pos;
uint16_t ODF_i2c;
uint16_t ODR_i2c;
uint16_t arm_i2c;

// Timer
hw_timer_t * timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter0;
hw_timer_t * timer1 = NULL;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter1;
int iTimer10;
int iTimer50;
int iTimer100;
int iTimer10_0;
bool timer10_flag = false;

// Time
char ntpServer[] = "ntp.nict.jp";
const long gmtOffset_sec = 9 * 3600;
const int  daylightOffset_sec = 0;
struct tm timeinfo;
String dateStr;
String timeStr;

unsigned long start_time;

//Xbee
char  xbee_re_buffer[16];
unsigned  int xbee_index;

unsigned char tx_pattern = 1;
unsigned char rx_pattern;
float re_val;

//RSSI
const int rssiPin = 34;
unsigned int rssi_width;
int rssi_value;
bool commu_loss_flag = false;
unsigned char commu_loss_cnt = 0;


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

uint8_t stamp_ota_number;
uint8_t stamp_ota_flag = 0;

//ToF
uint16_t tof_emg_dist;
uint16_t tof_target_dist;

//SD
File file;
String fname_buff;
const char* fname = "/haru_log.csv";

int32_t x;
int32_t y;

//main
unsigned int pattern = 0;
unsigned int sub_pattern = 0;
unsigned int lcd_pattern = 0;
int val = 0;
int i;
unsigned long last_time;
unsigned int  time_buff;
unsigned int  current_time;
int bb = 0;
int qq;
unsigned long can_time_buff;

int log_cnt = 0;
uint n = 0;
uint nmax = 0;
unsigned long microSD_Timer;
uint8_t encoder_status_counter;
uint8_t front_tof_status_counter;
uint8_t rear_tof_status_counter;
uint8_t emg_button_status_counter;
uint8_t acccel_distance;
uint8_t start_current;


//Flag
bool start_flag = false;
bool operation_flag = false;
bool Encoder_check_flag = false;
bool reset_flag = false;
bool IDLER_control_flag = false;
bool Lcd_flag = false;
bool log_flag = false;
bool microSD_rec_flag = false;
bool calculate_flag = false;
bool Encoder_emg_flag = false;
bool IDLER_Encoder_flag = false;
bool ODrive_vel_flag = false;
bool torque_flag = false;
bool ODrive_pos_flag = false;
bool emg_slip_flag = false;
bool emg_tof_flag = false;
bool emg_button_flag = false;
bool tof_pos_control_flag = false;
bool torque_control_flag = false;
bool main_ODreset_flag = false;
bool ODreset_flag = true;
bool ODrive_loop_flag = false;
bool ODmode_set_flag = true;
bool sub_torque_control_flag = false;
bool ODrive_state_send_flag = false;
bool ODrive_set_send_flag = false;
bool ODrive_send_flag = false;
bool current_control_flag = false;
bool xbee_flag = false;
bool i2c_flag = false;
bool IDLER_veL_control_flag = false;

void IRAM_ATTR onTimer0(void);
void IRAM_ATTR onTimer1(void);
void timerInterrupt0(void);
void timerInterrupt1(void);
void init_can();
void Recieve_can_F(void);
void sendData_F(int canId_esc, int flameType, int DLC, byte datas[]);
void VESC_Send_F(int id,int32_t input);
void Recieve_can_R(void);
void sendData_R(int canId_esc, int flameType, int DLC, byte datas[]);
void VESC_Send_R(int id,int32_t input);
void trajectory_control(float Xf, float Xi, float Vi,float Vmax, float Amax, float Dmax);
void position_control(float Xf, float Xi, float Vi);
void Lcd_display(void);
void Core2_OTA(void);
void taskDisplay(void *pvParameters);
void Recieve_stamp(void);
void Send_Stamp(int id,int dlc,byte data_s[]);
void getTimeFromNTP(void);
void getTime(void);
void set_log_name(void);
void eeprom_write(void);
void eeprom_read(void);
void xbee_re(void);
void xbee_se(void);

void setup()
{
  auto cfg = M5.config();
  cfg.internal_mic = false;
  cfg.external_imu = false;
  cfg.internal_imu = false;
  cfg.output_power = false;
  cfg.internal_spk = false;
  M5.begin(cfg);
  M5.Lcd.setTextSize(2);
  init_can(); 
  pinMode(EMG_PIN,INPUT);
  pinMode(ENC_PIN,OUTPUT);
  pinMode(rssiPin,INPUT);
  //pinMode(25,OUTPUT);
  // Initialize Timer Interrupt
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer1); 
  hand_brake_flag = true;
  Wire.begin(SDA_PIN,SCL_PIN,400000U);
  SD.begin(GPIO_NUM_4,SPI,12000000U);
  Serial2.begin(115200,SERIAL_8N1,13,14);
  delay(10);
  EEPROM.begin(128);
  delay(10);
  eeprom_read();
  delay(10);
  M5.Lcd.clear();
  M5.Lcd.fillScreen(BLACK);
  xTaskCreatePinnedToCore(&taskDisplay, "taskDisplay", 4096, NULL, 1, &task_handl, 0);
  digitalWrite(ENC_PIN,LOW);
  delay(10);

  if(hand_brake_current > 50)hand_brake_current = 50;
  if(brake_current > 50)brake_current = 50;
  M5.Lcd.drawJpgFile(SD,"/pic/000.jpg",0,0);
  lcd_pattern = 0;
  
}

void loop()
{
  if(OTA_flag) ArduinoOTA.handle();

  timerInterrupt1();

  switch(pattern){
    case 0:
      
      break;
    
    case 1:
      if(start_flag){
        target_pos = climb_height;
        erpm_lim = (float)climb_velocity * distance_trans;
        accel_lim = climber_accel;
        decel_lim = climber_decel;
        vel_lim = (float)climb_velocity / 3.6;
        start_flag = false;
        start_time = millis();   
        current_erpm_lim = 0;
        start_pos_F = odometer_buff_F;
        start_pos_R = odometer_buff_R;
        start_pos_Idree = travel_3_buffer;
        direction = true;
        M5.Lcd.clear();
        pattern = 11;
      }
      break;
        
    case 11:
      if(millis() - start_time > starting_count * 1000){
        start_time = millis();
        current_limit = start_current;
        last_change_dist = 0;
        dX = target_pos;
        accel_flag = true;
        current_brake_flag = false;
        hand_brake_flag = false;
        trajectory_control_flag = true;
        time_buff = millis();
        pattern = 21;
      }    
      break;
    
    case 21:
      if(millis() - time_buff > 1000 && !Encoder_check_flag){
        operation_flag = true;
        Encoder_check_flag = true;
      }

      if(cruise_flag){
        pattern = 22;
      }
      if(stop_dist > dX){
        accel_flag = false;
        decel_flag = true;
        deceleration_pos = current_pos;
        decel_flag = true;
        pattern = 23;
      }
      if(current_pos > target_pos){
        accel_flag = false;
        trajectory_control_flag = false;
        hand_brake_flag = true;
        operation_flag = false; 
        Encoder_check_flag = false; 
        time_buff = millis();
        pattern = 31;
      }
      break;
    
    case 22:
      if(millis() - time_buff > 1000 && !Encoder_check_flag){
        operation_flag = true;
        Encoder_check_flag = true;
      }
      if(stop_dist > dX){
        cruise_flag = false;
        decel_flag = true;
        deceleration_pos = current_pos;
        pattern = 23;
      }
      if(current_pos > target_pos){
        cruise_flag = false;
        trajectory_control_flag = false;
        operation_flag = false;
        hand_brake_flag = true;
        Encoder_check_flag = false; 
        operation_flag = false;
        time_buff = millis();
        pattern = 31;
      }
      break;
    
    case 23:
      if(millis() - time_buff > 1000 && !Encoder_check_flag){
        operation_flag = true;
        Encoder_check_flag = true;
      }
      if(position_control_flag){
        decel_flag = false;
        pattern = 24;
      }
      if(current_pos > target_pos){
        decel_flag = false;
        trajectory_control_flag = false;
        position_control_flag = false;
        current_brake_flag = false;
        operation_flag = false;
        hand_brake_flag = true;
        Encoder_check_flag = false; 
        time_buff = millis();
        pattern = 31;
      }
      break;
    
    case 24:
      if(millis() - time_buff > 1000 && !Encoder_check_flag){
        operation_flag = true;
        Encoder_check_flag = true;
      }
      if(!position_control_flag){ 
        operation_flag = false;
        current_brake_flag = false;
        hand_brake_flag = true;
        Encoder_check_flag = false; 
        time_buff = millis();
        pattern = 31;
      }
      break;
    
    case 31:
      if(millis() - time_buff > stop_wait * 1000){  
        target_pos = (float)down_margin + (float)low_speed_dist;
        erpm_lim = -1 * (((float)down_velocity * distance_trans));
        vel_lim = (float)down_velocity / 3.6;
        accel_lim = down_accel;
        decel_lim = down_decel;
        accel_flag = true;
        hand_brake_flag = false;
        time_buff = millis();
        dX = target_pos - current_pos; 
        trajectory_control_flag = true;
        current_erpm_lim = 0;
        Encoder_check_flag = false; 
        direction = false;
        pattern = 41;
      }
      break;
    
    case 41:
      if(millis() - time_buff > 1000 && !Encoder_check_flag){
        operation_flag = true;
        Encoder_check_flag = true;
      }
      if(cruise_flag){
        pattern = 42;
      }
      if(stop_dist < dX && operation_flag){
        accel_flag = false;
        decel_flag = true;
        deceleration_pos = climb_height - current_pos;
        decel_flag = true;
        pattern = 43;
      }

      if(current_pos < target_pos - (float)low_speed_dist){
        accel_flag = false;
        trajectory_control_flag = false;
        current_brake_flag = false;
        hand_brake_flag = true;
        operation_flag = false;       
        Encoder_check_flag = false; 
        time_buff = millis();
        finish_flag = true;
        pattern = 51;
      }
      break;
    
    case 42:
      if(millis() - time_buff > 1000 && !Encoder_check_flag){
        operation_flag = true;
        Encoder_check_flag = true;
      }
      if(stop_dist < dX && operation_flag ){
        cruise_flag = false;
        decel_flag = true;
        deceleration_pos = climb_height - current_pos;
        pattern = 43;
      }
      if(current_pos < target_pos - (float)low_speed_dist){
        cruise_flag = false;
        trajectory_control_flag = false;
        current_brake_flag = false;
        hand_brake_flag = true;
        operation_flag = false;
        Encoder_check_flag = false; 
        time_buff = millis();
        finish_flag = true;
        pattern = 51;
      }
      break;
    
    case 43:
      if(millis() - time_buff > 1000 && !Encoder_check_flag){
        operation_flag = true;
        Encoder_check_flag = true;
      }
      if(position_control_flag  || current_pos < (float)(down_margin + low_speed_dist)){
        target_pos = (float)down_margin;
        decel_flag = false;
        trajectory_control_flag = false;
        position_control_flag = true;
        pattern = 44;
      }

      if(current_pos < down_margin && trajectory_control_flag){
        decel_flag = false;
        trajectory_control_flag = false;
        current_brake_flag = false;
        hand_brake_flag = true;
        operation_flag = false;
        Encoder_check_flag = false; 
        time_buff = millis();
        finish_flag = true;
        pattern = 51;
      }
      break;
    
    case 44:
      if(millis() - time_buff > 1000 && !Encoder_check_flag){
        operation_flag = true;
        Encoder_check_flag = true;
      }
      if(!position_control_flag){
        position_control_flag = false;
        current_brake_flag = false;
        hand_brake_flag = true;
        operation_flag = false;
        time_buff = millis();
        Encoder_check_flag = false; 
        finish_flag = true;
        pattern = 51;
      }
      break;
    
    case 51:
      pattern = 52;
      break;
    
    case 52:
      break;

    
    case 61:
      if(start_flag){
        set_erpm = (float)Manual_climb_velocity * distance_trans;
        current_brake_flag = false;
        hand_brake_flag = false;
        velocity_control_flag = true;
        time_buff = millis();
        start_flag = false;
      }
      break;
    
    case 71:
      if(start_flag){
        set_erpm = -1 * ((float)Manual_Descend_velocity * distance_trans);
        current_brake_flag = false;
        hand_brake_flag = false;
        velocity_control_flag = true;
        time_buff = millis();
        start_flag = false;
      }
      break;
    
    case 200:
      if(reset_flag){
        start_flag = false;
        reset_flag = false;
        accel_flag = false;
        cruise_flag = false;
        decel_flag = false;
        velocity_control_flag = false;
        position_control_flag = false;
        current_brake_flag = false;
        trajectory_control_flag = false;
        hand_brake_flag = true;
        operation_flag = false;
        Encoder_check_flag = false;
        M5.Lcd.drawJpgFile(SD,"/pic/000.jpg",0,0);
        lcd_pattern = 0;
        pattern = 0;
      } 
      break;
    
    case 201:
      if(reset_flag){
        start_flag = false;
        reset_flag = false;
        accel_flag = false;
        cruise_flag = false;
        decel_flag = false;
        velocity_control_flag = false;
        position_control_flag = false;
        current_brake_flag = false;
        trajectory_control_flag = false;
        hand_brake_flag = true;
        operation_flag = false;
        Encoder_check_flag = false;
        M5.Lcd.drawJpgFile(SD,"/pic/000.jpg",0,0);
        lcd_pattern = 0;
        pattern = 0;
      } 
      break;

    
    case 203:
      if(reset_flag){
        start_flag = false;
        reset_flag = false;
        accel_flag = false;
        cruise_flag = false;
        decel_flag = false;
        velocity_control_flag = false;
        position_control_flag = false;
        current_brake_flag = false;
        trajectory_control_flag = false;
        operation_flag = false;
        Encoder_check_flag = false;
        M5.Lcd.drawJpgFile(SD,"/pic/000.jpg",0,0);
        lcd_pattern = 0;
        pattern = 0;
      } 
      break;
    
    case 204:
      if(reset_flag){
        start_flag = false;
        reset_flag = false;
        accel_flag = false;
        cruise_flag = false;
        decel_flag = false;
        velocity_control_flag = false;
        position_control_flag = false;
        current_brake_flag = false;
        trajectory_control_flag = false;
        operation_flag = false;
        Encoder_check_flag = false;
        M5.Lcd.drawJpgFile(SD,"/pic/000.jpg",0,0);
        lcd_pattern = 0;
        pattern = 0;
      } 
      break;
    
    case 205:
      if(reset_flag){
        start_flag = false;
        reset_flag = false;
        accel_flag = false;
        cruise_flag = false;
        decel_flag = false;
        velocity_control_flag = false;
        position_control_flag = false;
        current_brake_flag = false;
        trajectory_control_flag = false;
        operation_flag = false;
        Encoder_check_flag = false;
        M5.Lcd.drawJpgFile(SD,"/pic/000.jpg",0,0);
        lcd_pattern = 0;
        pattern = 0;
      } 
      break;


    
  } 


}

void taskDisplay(void *pvParameters){

  disableCore0WDT();

  while(1){      
    if(xbee_flag){
      xbee_re();
      xbee_se();
      xbee_flag = false;
    }
    
    if(i2c_flag){
    trans = abs(velocity_1 * 100.0);
    if(velocity_1 < 0){
      vel1 = 65535 - (uint32_t)trans;
    }else{
      vel1 = (uint32_t)trans;
    }
    
    trans = abs(velocity_2 * 100.0);
    if(velocity_2 < 0){
      vel2 = 65535 - (uint32_t)trans;
    }else{
      vel2 = (uint32_t)trans;
    }
    trans = abs(travel_1 * 100.0);
    if(travel_1 < 0){
      trv1 = 16777215 - (uint32_t)trans;
    }else{
      trv1 = (uint32_t)trans;
    }
    trans = abs(travel_2 * 100.0);
    if(travel_2 < 0){
      trv2 = 16777215 - (uint32_t)trans;
    }else{
      trv2 = (uint32_t)trans;
    }
    trans = abs(vesc_current_R);
    if(vesc_current_R < 0){
      current_R = 65535 - (uint32_t)trans;
    }else{
      current_R = (uint32_t)trans;
    }
    trans = abs(vesc_current_F);
    if(vesc_current_F < 0){
      current_F = 65535 - (uint32_t)trans;
    }else{
      current_F = (uint32_t)trans;
    }
    trans = abs(ODrive_Current_pos_F / GEAR_RATIO * SCREW_PITCH * 100.0);
    if(ODrive_Current_pos_F < 0){
      ODF_i2c = 65535 - (uint16_t)trans;
    }else{
      ODF_i2c = (uint16_t)trans;
    }
    trans = abs(ODrive_Current_pos_R / GEAR_RATIO * SCREW_PITCH * 100.0);
    if(ODrive_Current_pos_R < 0){
      ODR_i2c = 65535 - (uint16_t)trans;
    }else{
      ODR_i2c = (uint16_t)trans;
    }
    trans = abs(arm_current_pos * 100.0);
    if(arm_current_pos < 0){
      arm_i2c = 65535 - (uint16_t)trans;
    }else{
      arm_i2c = (uint16_t)trans;
    }
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write((uint8_t)(vel1 & 0xff));
    Wire.write((uint8_t)(vel1 >> 8 & 0xff));
    Wire.write((uint8_t)(vel2 & 0xff));
    Wire.write((uint8_t)(vel2 >> 8 & 0xff));
    Wire.write((uint8_t)(trv1 & 0xff));
    Wire.write((uint8_t)(trv1 >> 8 & 0xff));
    Wire.write((uint8_t)(trv1 >> 16 & 0xff));
    Wire.write((uint8_t)(trv2 & 0xff));
    Wire.write((uint8_t)(trv2 >> 8 & 0xff));
    Wire.write((uint8_t)(trv2 >> 16 & 0xff));
    Wire.write((uint8_t)(pattern & 0xff));
    Wire.write((uint8_t)(current_F & 0xff));
    Wire.write((uint8_t)(current_F >> 8 & 0xff));
    Wire.write((uint8_t)(current_R & 0xff));
    Wire.write((uint8_t)(current_R >> 8 & 0xff));
    Wire.write((uint8_t)((uint16_t)(vesc_voltage_F * 10.0)) & 0xff);
    Wire.write((uint8_t)((uint16_t)(vesc_voltage_F * 10.0)) >> 8 & 0xff);
    Wire.write((uint8_t)(avg_distance_F & 0xff));
    Wire.write((uint8_t)(avg_distance_F >> 8 & 0xff));
    Wire.write((uint8_t)(avg_distance_R & 0xff));
    Wire.write((uint8_t)(avg_distance_R >> 8 & 0xff));
    Wire.write((uint8_t)(ODF_i2c & 0xff));
    Wire.write((uint8_t)(ODF_i2c >> 8 & 0xff));
    Wire.write((uint8_t)(ODR_i2c & 0xff));
    Wire.write((uint8_t)(ODR_i2c >> 8 & 0xff));
    Wire.write((uint8_t)(arm_i2c & 0xff));
    Wire.write((uint8_t)(arm_i2c >> 8 & 0xff));
    Wire.endTransmission(true);
    delay(2);
    bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 7);
    i2c_flag  = false;
    }

    while(Wire.available()){
      vel_L = Wire.read();
      vel_H = Wire.read();
      trv_1 = Wire.read();
      trv_2 = Wire.read();
      trv_3 = Wire.read();
      arm_1 = Wire.read();
      arm_2 = Wire.read();
      calculate_flag = true;
    }
    if(calculate_flag){
      vel_buff = vel_L & 0xff | vel_H << 8 & 0xff00;
      if(vel_buff > 32767){
        velocity_3 = -1.0 * (float)(65535 - vel_buff) / 100.0 * IDLER_ROLLER_CON;//*************************** */
      }else{
        velocity_3 = (float)vel_buff / 100.0 * IDLER_ROLLER_CON;//*************************** */
      }
      trv_buff = trv_1 & 0xff | trv_2 << 8 & 0xff00 | trv_3 << 16 & 0xff0000;
      if(trv_buff > 8388608){
        travel_3_buffer = -1.0 * (float)(16777215 - trv_buff) / 100.0 * IDLER_ROLLER_CON;
        travel_3 = travel_3_buffer - start_pos_Idree;
      }else{
        travel_3_buffer = (float)trv_buff / 100.0 * IDLER_ROLLER_CON;
        travel_3 = travel_3_buffer - start_pos_Idree;
      }
      arm_buff = arm_1 & 0xff | arm_2 << 8 & 0xff00;
      if(arm_buff > 32768){
        arm_height = -1.0 * (float)(65535 - arm_buff) / 100.0;
        arm_current_pos = arm_height - arm_start_pos;
      }else{
        arm_height = (float)arm_buff / 100.0;
        arm_current_pos = arm_height - arm_start_pos;
      }
    }
    

    if(lcd_pattern == 21 || lcd_pattern == 22)
    Recieve_stamp();
    
  }
  
    
  
}
// Timer Interrupt Core1
//------------------------------------------------------------------//
void timerInterrupt1(void) {

  if (interruptCounter1 > 0) {

    portENTER_CRITICAL(&timerMux1);
    interruptCounter1--;
    portEXIT_CRITICAL(&timerMux1);
    
    
    Recieve_can_F();
    Recieve_can_R();
    Recieve_stamp();
    
    
    //val = !val;
    //digitalWrite(3,val);

    iTimer10++;
    //10ms timerInterrupt
    switch(iTimer10){
      case 1:
        i2c_flag = true;
        break;

      case 2:     
        xbee_flag = true;
        break;
      
      case 3:   
        if(IDLER_control_flag)
          current_pos = travel_3;
        else
          current_pos = travel_1;

        if(IDLER_veL_control_flag){
          current_vel = velocity_3 / 3.6;
        }else{
          if(vesc_rpm_F > vesc_rpm_R)
            current_vel = vesc_rpm_R / distance_trans /3.6;
          else
            current_vel = vesc_rpm_F / distance_trans /3.6;
        }

        trans = abs(arm_current_pos * 100.0);
        if(arm_current_pos < 0){
          arm = 65535 - (uint32_t)trans;
        }else{
          arm = (uint32_t)trans;
        }
        send_data[1] = arm & 0xff;
        send_data[2] = arm >> 8 & 0xff;

        if(trajectory_control_flag){
          if(target_pos > current_pos){
            s = 1.0;
          }else{
            s = -1.0;
          }
          trajectory_control(target_pos, current_pos, current_vel, vel_lim, accel_lim, decel_lim);  //m, m, m/s, m/s, m/s², m/s²
        
        }else if(velocity_control_flag){
          VESC_Send_F(INPUT_RPM,set_erpm);
          VESC_Send_R(INPUT_RPM,set_erpm);
        }else if(position_control_flag){
          position_control(target_pos, current_pos, current_vel);
        }else if(current_brake_flag){
          VESC_Send_F(INPUT_CURRENT_BRAKE,brake_current);
          VESC_Send_R(INPUT_CURRENT_BRAKE,brake_current);
        }else if(hand_brake_flag){
          VESC_Send_F(INPUT_HAND_BRAKE,hand_brake_current);
          VESC_Send_R(INPUT_HAND_BRAKE,hand_brake_current);
        }  
        
        break;
      
      case 4:
        send_data[0]++;
        if(send_data[0] > 255)send_data[0] = 0;
        Send_Stamp(Stamp_id,8,send_data);
        Stamp_id = 255;
        break;

      case 5:
        
        break;

      case 6:
        break;

      case 7:
        
        break;
      
      case 8:        
        break;

      case 10:
        if(pattern == 11){
          current_time = starting_count * 1000 - (millis() - start_time);
        }else if(pattern > 20){
          current_time = millis() - start_time;
        }else{
          current_time = 0;
        }
        timer10_flag = true;
        iTimer10 = 0;
        break;
    }

    iTimer50++;
    //50ms timerInterrupt
    switch(iTimer50){
      case 10:
        
        if(travel_3 == travel_3_buff){
          encoder_status_buffer.push(0);
        } else {
          encoder_status_buffer.push(1);
        }

        encoder_status_counter = 0;
        using index_t = decltype(encoder_status_buffer)::index_t;
        for (index_t i = 0; i < encoder_status_buffer.size(); i++) {
          encoder_status_counter += encoder_status_buffer[i];
        }

        if(Encoder_emg_flag && Encoder_check_flag && IDLER_Encoder_flag && encoder_status_counter <= 5 ) {
          Serial2.printf("\n\n");
          Serial2.printf(" Emergency Stop Encoder Count\n ");
          time_buff = millis();
          Stamp_id = EMG_STOP; 
          trajectory_control_flag = false;
          position_control_flag = false;
          velocity_control_flag = false;
          current_brake_flag = false;
          hand_brake_flag = true;
          start_flag = false;
          operation_flag = false;
          Encoder_check_flag = false; 
          time_buff = millis();
          pattern = 203;
          rx_pattern = 0;
          Serial2.printf("\n");
        }
        break;
      
      case 20:
        rssi_width = pulseIn(rssiPin, HIGH, 100);
        if( rssi_width == 0 ) {
          if( digitalRead(rssiPin) ) {
            rssi_value = -57;
          } else {
            rssi_value = 0;
          }
        } else {
          rssi_value = rssi_width * 1.3814 - 117;
        }      
        if( rssi_value < -90 ) {
          commu_loss_cnt++;
          if(commu_loss_cnt>=5) {
            commu_loss_flag = true;
          } 
        } else {
          commu_loss_flag = false;
          commu_loss_cnt = 0;
        }

        if(digitalRead(36)){
          emg_button_status_buffer.push(1);
        } else {
          emg_button_status_buffer.push(0);
        }

        emg_button_status_counter = 0;
        using index_t = decltype(emg_button_status_buffer)::index_t;
        for (index_t i = 0; i < emg_button_status_buffer.size(); i++) {
          emg_button_status_counter += emg_button_status_buffer[i];
        }

        if(emg_button_flag && ((pattern > 20 && pattern < 80)) && emg_button_status_counter >= 8 ) {
          Serial2.printf("\n\n");
          Serial2.printf(" Emergency Stop Button\n ");
          time_buff = millis();
          Stamp_id = EMG_STOP; 
          trajectory_control_flag = false;
          position_control_flag = false;
          velocity_control_flag = false;
          hand_brake_flag = false;
          current_brake_flag = true;
          start_flag = false;
          operation_flag = false;
          Encoder_check_flag = false; 
          time_buff = millis();
          pattern = 201;
          rx_pattern = 0;
          Serial2.printf("\n");
        }

        break;
      
      case 30:
        break;
      
      case 40:
        
        
        break;

      case 50:
        if(avg_distance_F != 0 && avg_distance_F < tof_emg_dist){
          front_tof_status_buffer.push(0);
        } else {
          front_tof_status_buffer.push(1);
        }

        front_tof_status_counter = 0;
        using index_t = decltype(front_tof_status_buffer)::index_t;
        for (index_t i = 0; i < front_tof_status_buffer.size(); i++) {
          front_tof_status_counter += front_tof_status_buffer[i];
        }

        if(emg_tof_flag && ((pattern > 20 && pattern < 30) || (pattern > 40 && pattern < 50)) && front_tof_status_counter <= 5 ) {
          Serial2.printf("\n\n");
          Serial2.printf(" Emergency Stop Front ToF Sensor\n ");
          time_buff = millis();
          Stamp_id = EMG_STOP; 
          trajectory_control_flag = false;
          position_control_flag = false;
          velocity_control_flag = false;
          hand_brake_flag = false;
          current_brake_flag = true;
          start_flag = false;
          operation_flag = false;
          Encoder_check_flag = false; 
          time_buff = millis();
          pattern = 204;
          rx_pattern = 0;
          Serial2.printf("\n");
        }

        if(avg_distance_R != 0 && avg_distance_R < tof_emg_dist){
          rear_tof_status_buffer.push(0);
        } else {
          rear_tof_status_buffer.push(1);
        }

        rear_tof_status_counter = 0;
        using index_t = decltype(rear_tof_status_buffer)::index_t;
        for (index_t i = 0; i < rear_tof_status_buffer.size(); i++) {
          rear_tof_status_counter += rear_tof_status_buffer[i];
        }

        if(emg_tof_flag && ((pattern > 20 && pattern < 30) || (pattern > 40 && pattern < 50)) && rear_tof_status_counter <= 5 ) {
          Serial2.printf("\n\n");
          Serial2.printf(" Emergency Stop Rear ToF Sensor\n ");
          time_buff = millis();
          Stamp_id = EMG_STOP; 
          trajectory_control_flag = false;
          position_control_flag = false;
          velocity_control_flag = false;
          hand_brake_flag = false;
          current_brake_flag = true;
          start_flag = false;
          operation_flag = false;
          Encoder_check_flag = false; 
          time_buff = millis();
          pattern = 205;
          rx_pattern = 0;
          Serial2.printf("\n");
        }
        iTimer50 = 0;
        break;
    }

    iTimer100++;
    //100ms timerInterrupt
    switch(iTimer100){
      case 10:
        if(pattern == 0)Lcd_display();
        break;
      
      case 60:
        if(tx_pattern ==  101 ){
          Serial2.printf("%5d, "  ,current_time);
          Serial2.printf("%3d, "  ,pattern);
          Serial2.printf("%3d, "  ,sub_pattern);
          Serial2.printf("%3.2f, "  ,set_speed);
          Serial2.printf("%3.2f, "  ,velocity_1);
          Serial2.printf("%4.2f, "  ,travel_1);
          Serial2.printf("%3.2f, "  ,velocity_2);
          Serial2.printf("%4.2f, "  ,travel_2);
          Serial2.printf("%3.2f, "  ,velocity_3);
          Serial2.printf("%4.2f, "  ,travel_3);
          Serial2.printf("%5.2f, "  ,ODrive_Current_pos_F / GEAR_RATIO * SCREW_PITCH);
          Serial2.printf("%5.2f, "  ,ODrive_Current_pos_R / GEAR_RATIO * SCREW_PITCH);
          Serial2.printf("%5.2f, "  ,-1.0 * arm_current_pos);
          
        }
        break;
      
      case 70:
        if(tx_pattern ==  101 ){
          Serial2.printf("%4d, "  ,Current_State_F);
          Serial2.printf("%4d, "  ,Current_State_R);
          Serial2.printf("%5d, "  ,avg_distance_F);
          Serial2.printf("%5d, "  ,avg_distance_R);
          Serial2.printf("%4.1f, ",vesc_current_F / 10.0); 
          Serial2.printf("%4.1f, ",vesc_current_R/10.0); 
          Serial2.printf("%5d, "  ,rssi_value);
          Serial2.printf("\n");
        }
        break;

      case 80:
        break;

      case 200:
        iTimer100 = 0;
        break;
    }
      


  }
}
void init_can()
{
  SPIClass mffVSPI = SPIClass(VSPI);
  SPI.begin(VSPI_CLOCK_PIN, VSPI_MISO_PIN, VSPI_MOSI_PIN);
  CAN1.begin(SPI,MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
  CAN1.setMode(MCP_NORMAL);
  pinMode(CAN1_INT, INPUT);
  CAN2.begin(SPI,MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
  CAN2.setMode(MCP_NORMAL);
  pinMode(CAN2_INT, INPUT);
  CAN3.begin(SPI,MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
  CAN3.setMode(MCP_NORMAL);
  pinMode(CAN3_INT, INPUT);
}
void Send_Stamp(int id,int dlc,byte data_s[]){
  CAN3.sendMsgBuf(id, TYPE1, dlc, data_s);
}
void Recieve_stamp(void)
{
  if(!digitalRead(CAN3_INT))
  {
    CAN3.readMsgBuf(&canId, &len, datas);

    if(canId ==0x100){
      
      ODrive_F[0] = (uint8_t)datas[0];
      ODrive_F[1] = (uint8_t)datas[1];
      ODrive_F[2] = (uint8_t)datas[2];
      ODrive_R[0] = (uint8_t)datas[3];
      ODrive_R[1] = (uint8_t)datas[4];
      ODrive_R[2] = (uint8_t)datas[5];
      Current_State_F = (uint8_t)datas[6];
      Current_State_R = (uint8_t)datas[7];


      ODrive_F_buff = ODrive_F[0] & 0xff | (ODrive_F[1] << 8) & 0xff00 | (ODrive_F[2] << 16) & 0xff0000;
      if(ODrive_F_buff > 0x800000){
        ODrive_Current_pos_F = -1.0 * (float)(0xffffff - ODrive_F_buff) / 100.0;
      }else{
        ODrive_Current_pos_F = (float)ODrive_F_buff / 100.0;
      }

      ODrive_R_buff = ODrive_R[0] & 0xff | (ODrive_R[1] << 8) & 0xff00 | (ODrive_R[2] << 16) & 0xff0000;
      if(ODrive_R_buff > 0x800000){
        ODrive_Current_pos_R = -1.0 * (float)(0xffffff - ODrive_R_buff) / 100.0;
      }else{
        ODrive_Current_pos_R = (float)ODrive_R_buff / 100.0;
      }

      }

    

    if(canId==0x101){
      for (int i = 0; i < 8; i++)
      {
        switch (i)
        {
          case 0:
            dis1 = (uint8_t)datas[i];
            break;
          case 1:
            dis2 = (uint8_t)datas[i];
            break;
          case 2:
            dis3 = (uint8_t)datas[i];
            break;
          case 3:
            dis4 = (uint8_t)datas[i];
            break;
          case 4:
            sub_pattern = (uint8_t)datas[i];
            break;
          case 5:
            ODrive_vbus = (float)datas[i] / 10.0;
            break;
        }
      }
      avg_distance_F = dis1 & 0xff | dis2 << 8 & 0xff00;
      avg_distance_R = dis3 & 0xff | dis4 << 8 & 0xff00;
    }
  }
}


//------------------------------------------------------------------//
void VESC_Send_F(int id,int32_t input){
  uint32_t input_data;
  int32_t set_data = input;
  uint8_t buffer[8];
  int total_id;

  total_id = id << 8 | VESC_ID_F;

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
      sendData_F(total_id, TYPE2, DLC2, buffer);
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
      sendData_F(total_id, TYPE2, DLC2, buffer);
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
      sendData_F(total_id, TYPE2, DLC2, buffer);
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
      sendData_F(total_id, TYPE2, DLC2, buffer);
      break;
    
    case 4:
      input_data = set_data * 1000000;
      buffer[0] = (input_data >> 24) & 0xFF;
      buffer[1] = (input_data >> 16) & 0xFF;
      buffer[2] = (input_data  >> 8  ) & 0xFF;
      buffer[3] = input_data & 0xFF;
      sendData_F(total_id, TYPE2, DLC2, buffer);
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
      sendData_F(total_id, TYPE2, DLC2, buffer);
    break;
    
  }
}
//CAN Send
//------------------------------------------------------------------//
void sendData_F(int canId_esc, int flameType, int DLC, byte datas[]){
  CAN1.sendMsgBuf(canId_esc, flameType, DLC, datas);
  String data = "None";
}
void Recieve_can_F(){
  String data = "None";
  if(!digitalRead(CAN1_INT))
  {
    CAN1.readMsgBuf(&canId_vesc, &len, datas);

    if(canId_vesc==(VESC_ID_F| 0x80000900)){
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

        if(duty_buff >= 32768) {
          vesc_duty = (duty_buff - 65535) / 10;
        } else {
          vesc_duty = duty_buff / 10;          
        }
        if(current_buff >= 32768) {
          vesc_current_F = (current_buff - 65535) ;
        } else {
          vesc_current_F = current_buff;          
        }
        if(rpm_buff >= 0x80000000) {
          vesc_rpm_F = (rpm_buff - 0xFFFFFFFF);
        } else {
          vesc_rpm_F = rpm_buff;          
        }
        velocity_1 = (float)vesc_rpm_F / distance_trans;
    }
      
    if(canId_vesc==(VESC_ID_F| 0x80000e00)){
      for(i = 0; i < 8; i++){
          switch (i) {
          case 0:
            VESC.amph_3 = (char)datas[i];
            break;
          case 1:
            VESC.amph_2 = (char)datas[i];
            break;
          case 2:
            VESC.amph_1 = (char)datas[i];
            break;
          case 3:
            VESC.amph_0 = (char)datas[i];
            break; 
          case 4:
            VESC.amphc_3 = (char)datas[i];
            break;          
          case 5:
            VESC.amphc_2 = (char)datas[i];
            break;
          case 6:
            VESC.amphc_1 = (char)datas[i];
            break;
          case 7:
            VESC.amphc_0 = (char)datas[i];
            break;       
          }
        }  
      
        amp_hours = ((VESC.amph_3 << 24 & 0xFF000000) | (VESC.amph_2 << 16 & 0x00FF0000) | (VESC.amph_1 << 8 & 0x0000FF00) | (VESC.amph_0 & 0xFF));
        amph_hours_charged = ((VESC.amphc_3 << 24 & 0xFF000000) | (VESC.amphc_2 << 16 & 0x00FF0000) | (VESC.amphc_1 << 8 & 0x0000FF00) | (VESC.amphc_0 & 0xFF));
    }

    if(canId_vesc==(VESC_ID_F| 0x80001000)){
      for(i = 0; i < 8; i++){
          switch (i) {
          case 0:
            VESC.fet_temp_H = (char)datas[i];
            break;
          case 1:
            VESC.fet_temp_L = (char)datas[i];
            break;
          case 2:
            VESC.motor_temp_H = (char)datas[i];
            break;
          case 3:
            VESC.motor_temp_L = (char)datas[i];
            break; 
          case 4:
            VESC.total_current_H = (char)datas[i];
            break;          
          case 5:
            VESC.total_current_L = (char)datas[i];
            break;
          case 6:
            VESC.pos_H = (char)datas[i];
            break;
          case 7:

            VESC.pos_L = (char)datas[i];
            break;       
          }
        }  
        fet_temp = ((VESC.fet_temp_H << 8 & 0xFF00) | (VESC.fet_temp_L & 0xFF)) / 10;
        motor_temp = ((VESC.motor_temp_H << 8 & 0xFF00) | (VESC.motor_temp_L & 0xFF)) / 10;
        total_current = ((VESC.total_current_H << 8 & 0xFF00) | (VESC.total_current_L & 0xFF)) / 10;
        current_position = ((VESC.pos_H << 8 & 0xFF00) | (VESC.pos_L & 0xFF)) / 50;
    }

    if(canId_vesc==(VESC_ID_F| 0x80001b00)){
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
        odometer_buff_F = ((VESC.odo_3 << 24 & 0xFF000000) | (VESC.odo_2 << 16 & 0x00FF0000) | (VESC.odo_1 << 8 & 0x0000FF00) | (VESC.odo_0 & 0xFF));
        travel_1 = ((float)odometer_buff_F - start_pos_F) / (MOTOR_POLE_PAIRS * 2.0 * 3.0) * DRIVER_ROLLER_PERIMETER / 1000.0;
        vesc_voltage_F = (float)((VESC.voltage_H << 8 & 0xFF00) | (VESC.voltage_L & 0xFF)) / 10.0;
         
    }
  }
}
//------------------------------------------------------------------//
void VESC_Send_R(int id,int32_t input){
  uint32_t input_data;
  int32_t set_data = input;
  uint8_t buffer[8];
  int total_id;

  total_id = id << 8 | VESC_ID_R;

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
      sendData_R(total_id, TYPE2, DLC2, buffer);
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
      sendData_R(total_id, TYPE2, DLC2, buffer);
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
      sendData_R(total_id, TYPE2, DLC2, buffer);
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
      sendData_R(total_id, TYPE2, DLC2, buffer);
      break;
    
    case 4:
      input_data = set_data * 1000000;
      buffer[0] = (input_data >> 24) & 0xFF;
      buffer[1] = (input_data >> 16) & 0xFF;
      buffer[2] = (input_data  >> 8  ) & 0xFF;
      buffer[3] = input_data & 0xFF;
      sendData_R(total_id, TYPE2, DLC2, buffer);
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
      sendData_R(total_id, TYPE2, DLC2, buffer);
    break;
    
  }
}
//CAN Send
//------------------------------------------------------------------//
void sendData_R(int canId_esc, int flameType, int DLC, byte datas[]){
  CAN2.sendMsgBuf(canId_esc, flameType, DLC, datas);
  String data = "None";
}
void Recieve_can_R(){
  String data = "None";
  if(!digitalRead(CAN2_INT))
  {
    CAN2.readMsgBuf(&canId_vesc, &len, datas);

    if(canId_vesc==(VESC_ID_R| 0x80000900)){
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

        if(duty_buff >= 32768) {
          vesc_duty = (duty_buff - 65535) / 10;
        } else {
          vesc_duty = duty_buff / 10;          
        }
        if(current_buff >= 32768) {
          vesc_current_R = (current_buff - 65535) ;
        } else {
          vesc_current_R = current_buff;          
        }
        if(rpm_buff >= 0x80000000) {
          vesc_rpm_R = (rpm_buff - 0xFFFFFFFF);
        } else {
          vesc_rpm_R = rpm_buff;          
        }
        velocity_2 = (float)vesc_rpm_R / distance_trans;
    }
      
    if(canId_vesc==(VESC_ID_R| 0x80000e00)){
      for(i = 0; i < 8; i++){
          switch (i) {
          case 0:
            VESC.amph_3 = (char)datas[i];
            break;
          case 1:
            VESC.amph_2 = (char)datas[i];
            break;
          case 2:
            VESC.amph_1 = (char)datas[i];
            break;
          case 3:
            VESC.amph_0 = (char)datas[i];
            break; 
          case 4:
            VESC.amphc_3 = (char)datas[i];
            break;          
          case 5:
            VESC.amphc_2 = (char)datas[i];
            break;
          case 6:
            VESC.amphc_1 = (char)datas[i];
            break;
          case 7:
            VESC.amphc_0 = (char)datas[i];
            break;       
          }
        }  
      
        amp_hours = ((VESC.amph_3 << 24 & 0xFF000000) | (VESC.amph_2 << 16 & 0x00FF0000) | (VESC.amph_1 << 8 & 0x0000FF00) | (VESC.amph_0 & 0xFF));
        amph_hours_charged = ((VESC.amphc_3 << 24 & 0xFF000000) | (VESC.amphc_2 << 16 & 0x00FF0000) | (VESC.amphc_1 << 8 & 0x0000FF00) | (VESC.amphc_0 & 0xFF));
    }

    if(canId_vesc==(VESC_ID_R| 0x80001000)){
      for(i = 0; i < 8; i++){
          switch (i) {
          case 0:
            VESC.fet_temp_H = (char)datas[i];
            break;
          case 1:
            VESC.fet_temp_L = (char)datas[i];
            break;
          case 2:
            VESC.motor_temp_H = (char)datas[i];
            break;
          case 3:
            VESC.motor_temp_L = (char)datas[i];
            break; 
          case 4:
            VESC.total_current_H = (char)datas[i];
            break;          
          case 5:
            VESC.total_current_L = (char)datas[i];
            break;
          case 6:
            VESC.pos_H = (char)datas[i];
            break;
          case 7:
            VESC.pos_L = (char)datas[i];
            break;       
          }
        }  
        fet_temp = ((VESC.fet_temp_H << 8 & 0xFF00) | (VESC.fet_temp_L & 0xFF)) / 10;
        motor_temp = ((VESC.motor_temp_H << 8 & 0xFF00) | (VESC.motor_temp_L & 0xFF)) / 10;
        total_current = ((VESC.total_current_H << 8 & 0xFF00) | (VESC.total_current_L & 0xFF)) / 10;
        current_position = ((VESC.pos_H << 8 & 0xFF00) | (VESC.pos_L & 0xFF)) / 50;
    }

    if(canId_vesc==(VESC_ID_R| 0x80001b00)){
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
        odometer_buff_R = ((VESC.odo_3 << 24 & 0xFF000000) | (VESC.odo_2 << 16 & 0x00FF0000) | (VESC.odo_1 << 8 & 0x0000FF00) | (VESC.odo_0 & 0xFF));
        travel_2 = ((float)odometer_buff_R - start_pos_R)/ (MOTOR_POLE_PAIRS * 2.0 * 3.0) * DRIVER_ROLLER_PERIMETER / 1000.0;
        vesc_voltage_R = (float)((VESC.voltage_H << 8 & 0xFF00) | (VESC.voltage_L & 0xFF)) / 10.0;
         
    }
  }
}

//--------------------------------------------------------------------------------------------//
void trajectory_control(float Xf, float Xi, float Vi,float Vmax, float Amax, float Dmax) {
  
  
  V_max = s * Vmax;
  D_max = s * Dmax;
  A_max = s * Amax;
  dX = Xf - Xi;
  stop_dist = ((Vi * Vi) / (2.0f * D_max)); 
  position_control_erpm = s * (position_control_velocity * distance_trans);
  position_control_down_erpm = s * (position_control_down_velocity * distance_trans);

  if(accel_flag){
    if(current_control_flag && pattern == 21){
      Vo = Vi;
      if(Xi - last_change_dist > (float)acccel_distance){
        current_limit = current_limit + 1;
        if(current_limit > throttle_current){
          current_limit = throttle_current;
        }
        last_change_dist = Xi;
      }
      VESC_Send_F(INPUT_CURRENT,(int32_t)current_limit);
      VESC_Send_R(INPUT_CURRENT,(int32_t)current_limit);
    }else{
      Vo = Vi;
      current_erpm_lim += float(A_max * 3.6 * distance_trans) / CONTROL_FREQUENCY;
      if( current_erpm_lim > erpm_lim && s > 0){
        current_erpm_lim = erpm_lim;
        accel_flag = false;
        cruise_flag = true;
      }else if(current_erpm_lim < erpm_lim && s < 0){
        current_erpm_lim = erpm_lim;
        accel_flag = false;
        cruise_flag = true;
      }
    }
  }
  
  if(cruise_flag){
    Vo = Vi;
  }

  if(decel_flag){    
    if(s > 0){
      //current_erpm_lim = sqrt((-2.0 * D_max * (Xi - deceleration_pos)) + (Vo * Vo)) * 3.6 * distance_trans;
      Dmax_cor = (Vo * Vo) / (2 * (Xf - deceleration_pos));
      current_erpm_lim = sqrt((-2.0 * Dmax_cor * (Xi - deceleration_pos)) + (Vo * Vo)) * 3.6 * distance_trans;
      if(current_erpm_lim < position_control_erpm){
        trajectory_control_flag = false;
        position_control_flag = true;
      }
    }else{
      //float down_x = (float)climb_height - Xi;
      //current_erpm_lim = -1 * sqrt((2.0 * D_max * (deceleration_pos - Xi )) + (Vo * Vo)) * 3.6 * distance_trans;
      current_erpm_lim += (Dmax * 3.6 * distance_trans) / CONTROL_FREQUENCY; 
      if(current_erpm_lim > (float)position_control_down_erpm  || Xi < (float)down_margin + (float)low_speed_dist){
        target_pos = (float)down_margin;
        trajectory_control_flag = false;
        position_control_flag = true;
      }
    }
  }  
  if((pattern > 20 && pattern < 30) && current_pos > target_pos){
    current_erpm_lim = 0;
  }else if((pattern > 40 && pattern < 50) && current_pos < target_pos){
    current_erpm_lim = 0;
  }
  set_speed = current_erpm_lim / distance_trans;
  if(!current_control_flag || (current_control_flag && !accel_flag)){
    VESC_Send_F(INPUT_RPM,(int32_t)current_erpm_lim);
    VESC_Send_R(INPUT_RPM,(int32_t)current_erpm_lim);
  }
  

}

void position_control(float Xf, float Xi, float Vi){
  float s;
  if(Xf > Xi){
    s = 1.0;
  }else{
    s = -1.0;
  }
  qq = s;
  position_control_erpm = s * (position_control_velocity * distance_trans);
  position_control_down_erpm = s * (position_control_down_velocity * distance_trans);

  float integral = 0;
  float last_err = 0;
  unsigned long last_micros = 0;

  float dX = Xf - Xi;  
  float P = pos_p * dX * SCALE;
  unsigned long current_micros = micros();            
  float dt = ((float)(current_micros - last_micros))
              / 1000000.0; 
  
  integral += dX * dt;                              
  float I = pos_i * integral * SCALE;                       
  float diff = (dX - last_err) / dt;                
  float D = pos_d * diff * SCALE;                           
  current_erpm_lim = P + I + D;

  if(s > 0){
    if(current_erpm_lim > (float)position_control_erpm){
      current_erpm_lim = (float)position_control_erpm;
    }
    if(current_erpm_lim < MINIMUM_ERPM){
      current_erpm_lim = 0;
    }
  }else{
    if(current_erpm_lim < (float)position_control_down_erpm){
      current_erpm_lim = (float)position_control_down_erpm;
    }
    if(current_erpm_lim >  -1.0 * MINIMUM_ERPM){
      current_erpm_lim = 0;
    }
  }
  
  if((pattern > 20 && pattern < 30) && ((current_pos > target_pos) || (Vi < 0.1 ))){
    current_erpm_lim = 0;
    position_control_flag = false;
  }else if((pattern > 40 && pattern < 50) && ((current_pos < target_pos) || (Vi > -0.1 ))){
    current_erpm_lim = 0;
    position_control_flag = false;
  }
  set_speed = current_erpm_lim / distance_trans;
  VESC_Send_F(INPUT_RPM,current_erpm_lim);
  VESC_Send_R(INPUT_RPM,current_erpm_lim);

  last_err = dX;                                    
  last_micros = current_micros;  

}

void Lcd_display(void){
  int16_t x,y;
  bool press;
  bool release_flag = true;
  bool draw_flag = true;
  int nums;

  M5.update();
  if (M5.Touch.isEnabled()) {
    auto t = M5.Touch.getDetail();
    x = t.prev_x;
    y = t.prev_y;
    press = t.isPressed();
  }
  
  if(millis() - time_buff > 800)release_flag = true;
  switch(lcd_pattern){
    case 0:
      if(draw_flag){
        M5.Lcd.drawJpgFile(SD,"/pic/0.jpg",0,0);
        draw_flag = false;
      }
      if(press && release_flag){
        M5.Lcd.clear();
        time_buff = millis();
        release_flag = false;
        if(x < 160 && y < 120){
          M5.Lcd.drawJpgFile(SD,"/pic/1.jpg",0,0);
          lcd_pattern = 1;
        }else if(x < 160 && y >= 120){
          M5.Lcd.drawJpgFile(SD,"/pic/2.jpg",0,0);
          lcd_pattern = 2;
        }else if(x >= 160 && y < 120){
          M5.Lcd.drawJpgFile(SD,"/pic/3.jpg",0,0);
          lcd_pattern = 3;
        }else if(x >= 160 && y >= 120){
          M5.Lcd.drawJpgFile(SD,"/pic/4.jpg",0,0);
          lcd_pattern = 4;
        }
      }
      break;
    case 1: //OTA
      if(press){
        if(x > 160 && y < 120){
          M5.Lcd.clear();
          time_buff = millis();
          release_flag = false;
          M5.Lcd.drawJpgFile(SD,"/pic/11.jpg",0,0);
          lcd_pattern = 11;
          Core2_OTA();
        }else if(x > 160 && y >= 120){
          M5.Lcd.clear();
          time_buff = millis();
          release_flag = false;
          Send_Stamp(STAMP_OTA,1,data_stamp);
          M5.Lcd.drawJpgFile(SD,"/pic/15.jpg",0,0);
          lcd_pattern = 12;
        }else if(x < 120 && y >= 170){
          M5.Lcd.clear();
          time_buff = millis();
          release_flag = false;
          draw_flag = true;
          lcd_pattern = 0;
        }
      }
      break;
    
    case 2: //TOF
      if(press){
        if(x < 160 && y < 170){
          M5.Lcd.clear();
          time_buff = millis();
          release_flag = false;
          M5.Lcd.drawJpgFile(SD,"/pic/21.jpg",0,0);
          command_id = 21;
          lcd_pattern = 21;
        }else if(x > 160 && y < 170){
          M5.Lcd.clear();
          time_buff = millis();
          release_flag = false;
          M5.Lcd.drawJpgFile(SD,"/pic/22.jpg",0,0);
          command_id = 22;
          lcd_pattern = 22;
        }else if(x < 120 && y >= 170){
          M5.Lcd.clear();
          time_buff = millis();
          release_flag = false;
          draw_flag = true;
          lcd_pattern = 0;
        }
      }
      break;
    
    case 3: //Voltage
      Stamp_id = 20;
      M5.Display.startWrite(); 
      M5.Display.setTextSize(2.5); 
      M5.Display.setCursor(30, 80);
      M5.Display.printf("VESC = %02.1f V",vesc_voltage_F);
      M5.Display.setCursor(30, 140);
      M5.Display.printf("ODrive = %02.1f V",ODrive_vbus);
      M5.Display.endWrite();
      if(press){
        if(x < 120 && y >= 170){
          M5.Lcd.clear();
          time_buff = millis();
          release_flag = false;
          draw_flag = true;
          lcd_pattern = 0;
        }
      }
      break;
    
    case 4: //Pressure
      M5.Lcd.setTextSize(2.5);  
      M5.Lcd.setCursor(20, 100);  
      //M5.Lcd.printf("Altitude =  %07.2f",alt_avg); 
      
      if(press){
        if(x < 120 && y >= 170){
          M5.Lcd.clear();
          time_buff = millis();
          release_flag = false;
          draw_flag = true;
          lcd_pattern = 0;
        }else if(x > 180 && y > 170){
          Send_Stamp(STAMP_INIT_BARO,1,data_stamp);
        }
      }
      break;
    
    case 11: 
      if(OTA_flag)
        M5.Lcd.drawJpgFile(SD,"/pic/12.jpg",0,0);
      else
        M5.Lcd.drawJpgFile(SD,"/pic/11.jpg",0,0);
      if(press){
        if(x < 120 && y >= 170){
          M5.Lcd.clear();
          time_buff = millis();
          release_flag = false;
          OTA_flag = false;
          WiFi.disconnect();
          M5.Lcd.drawJpgFile(SD,"/pic/1.jpg",0,0);
          lcd_pattern = 1;
        }
      }
      break;
    
    case 12: 
      if(stamp_ota_flag){
        M5.Lcd.clear();
        M5.Lcd.drawJpgFile(SD,"/pic/14.jpg",0,0);
        lcd_pattern = 13;
      }
      
      if(press){
        if(x < 120 && y >= 170){
          M5.Lcd.clear();
          time_buff = millis();
          release_flag = false;
          Send_Stamp(STAMP_OTA_END,1,data_stamp);
          M5.Lcd.drawJpgFile(SD,"/pic/1.jpg",0,0);
          stamp_ota_flag = 0;
          lcd_pattern = 1;
        }
      }
      break;
    
    case 13:
      if(press){
        if(x < 120 && y >= 170){
          M5.Lcd.clear();
          time_buff = millis();
          release_flag = false;
          Send_Stamp(STAMP_OTA_END,1,data_stamp);
          M5.Lcd.drawJpgFile(SD,"/pic/1.jpg",0,0);
          stamp_ota_flag = 0;
          lcd_pattern = 1;
        }
      }
      break;
    
    case 21: 
      M5.Lcd.setTextSize(2);  
      M5.Lcd.setCursor(160, 40);  
      M5.Lcd.printf("AVG = %04d",avg_distance_F); 
      M5.Lcd.setCursor(50, 80);  
      M5.Lcd.printf("%04d   %04d   %04d",distance1,distance2,distance3); 
      M5.Lcd.setCursor(50, 125);  
      M5.Lcd.printf("%04d   %04d   %04d",distance4,distance5,distance6); 
      M5.Lcd.setCursor(50, 170);  
      M5.Lcd.printf("%04d   %04d   %04d",distance7,distance8,distance9); 
      
      if(press){
        if(x < 120 && y >= 170){
          M5.Lcd.clear();
          time_buff = millis();
          release_flag = false;
          M5.Lcd.drawJpgFile(SD,"/pic/2.jpg",0,0);
          command_id = 0;
          lcd_pattern = 2;
        }
      }
      break;
    
    case 22: 
      M5.Lcd.setTextSize(2);  
      M5.Lcd.setCursor(160, 40);  
      M5.Lcd.printf("AVG = %04d",avg_distance_R); 
      M5.Lcd.setCursor(50, 80);  
      M5.Lcd.printf("%04d   %04d   %04d",distance1,distance2,distance3); 
      M5.Lcd.setCursor(50, 125);  
      M5.Lcd.printf("%04d   %04d   %04d",distance4,distance5,distance6); 
      M5.Lcd.setCursor(50, 170);  
      M5.Lcd.printf("%04d   %04d   %04d",distance7,distance8,distance9); 
      if(press){
        if(x < 120 && y >= 170){
          M5.Lcd.clear();
          time_buff = millis();
          release_flag = false;
          M5.Lcd.drawJpgFile(SD,"/pic/2.jpg",0,0);
          command_id = 0;
          lcd_pattern = 2;
        }
      }
      break;
    
  }
  
}
void Core2_OTA(void){
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 80);
  M5.Lcd.printf("Connecting Home");  
  wifi_cnt = 0;
  WiFi.mode(WIFI_STA); 
  WiFi.begin(ssid1, pass1);  
  while (WiFi.status() != WL_CONNECTED) {
    wifi_cnt++;
    M5.Lcd.printf(".");  
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
    M5.Lcd.clear();
    M5.Lcd.drawJpgFile(SD,"/pic/5.jpg",0,0);
    M5.Lcd.setCursor(0, 80);  
    M5.Lcd.printf("Connecting PC ");  
    WiFi.mode(WIFI_STA); 
    WiFi.begin(ssid2, pass2);  
    while (WiFi.status() != WL_CONNECTED) {
      wifi_cnt++;
      M5.Lcd.printf(".");  
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
    M5.Lcd.clear();
    M5.Lcd.drawJpgFile(SD,"/pic/5.jpg",0,0);
    M5.Lcd.setCursor(0, 80);  
    M5.Lcd.printf("Connecting Phone ");  
    WiFi.mode(WIFI_STA); 
    WiFi.begin(ssid3, pass3);  
    while (WiFi.status() != WL_CONNECTED) {
      wifi_cnt++;
      M5.Lcd.printf(".");  
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
  .setHostname("M5Core2")
  .onStart([]() {})
  .onEnd([]() {})
  .onProgress([](unsigned int progress, unsigned int total) {})
  .onError([](ota_error_t error) {}); 
  ArduinoOTA.begin();
}
// IRAM
//------------------------------------------------------------------//
void IRAM_ATTR onTimer1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  interruptCounter1=1;
  portEXIT_CRITICAL_ISR(&timerMux1);
}
void IRAM_ATTR onTimer0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  interruptCounter0=1;
  portEXIT_CRITICAL_ISR(&timerMux0);
}
//Get Time From NTP
//------------------------------------------------------------------//
void getTimeFromNTP(void){
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  while (!getLocalTime(&timeinfo)) {
    delay(1000);
  }
}

//Get Convert Time
//------------------------------------------------------------------//
void getTime(void){
  getLocalTime(&timeinfo);
  dateStr = (String)(timeinfo.tm_year + 1900)
          + "/" + (String)(timeinfo.tm_mon + 1)
          + "/" + (String)timeinfo.tm_mday;
  timeStr = (String)timeinfo.tm_hour
          + ":" + (String)timeinfo.tm_min
          + ":" + (String)timeinfo.tm_sec;
}


//XBee RX
//------------------------------------------------------------------//
void xbee_re(void){

    while(Serial2.available()){
      xbee_re_buffer[xbee_index]=Serial2.read();
      Serial2.write(xbee_re_buffer[xbee_index]);

      if(xbee_re_buffer[xbee_index]==0x08){
        xbee_re_buffer[xbee_index-1]=0;
        xbee_index--;
        Serial2.printf(" ");
        Serial2.write(0x08);
      }else if(xbee_re_buffer[xbee_index]==0x0D){
        Serial2.read();
        Serial2.printf("\n\n");
        if(tx_pattern == 0){
          rx_pattern = atoi(xbee_re_buffer);
        }else if(tx_pattern == 5){
          re_val = atof(xbee_re_buffer);
        }
        xbee_index = 0;

        switch(rx_pattern){
        case 0:
          tx_pattern = 1;
          break;
        case 1:
          pattern = 1;
          tx_pattern = 1;
          break;
        case 2:
          tx_pattern = 2;
          break;
        case 3:
          tx_pattern = 3;
          break;

        case 31:
          tx_pattern = 31;
          rx_pattern = 71;
          break;
        case 71:
          climb_height = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 32:
          tx_pattern = 32;
          rx_pattern = 72;
          break;
        case 72:
          climb_velocity = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 33:
          tx_pattern = 33;
          rx_pattern = 73;
          break;
        case 73:
          climber_accel = re_val;
          climber_accel_ROM = climber_accel * 10.0;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 34:
          tx_pattern = 34;
          rx_pattern = 74;
          break;
        case 74:
          climber_decel = re_val;
          climber_decel_ROM = climber_decel * 10.0;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 35:
          tx_pattern = 35;
          rx_pattern = 75;
          break;
        case 75:
          down_velocity = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 36:
          tx_pattern = 36;
          rx_pattern = 76;
          break;
        case 76:
          down_accel = re_val;
          down_accel_ROM = down_accel * 10.0;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 37:
          tx_pattern = 37;
          rx_pattern = 77;
          break;
        case 77:
          down_decel= re_val;
          down_decel_ROM = down_decel * 10.0;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 38:
          tx_pattern = 38;
          rx_pattern = 78;
          break;
        case 78:
          pos_p= re_val;
          pos_p_ROM = pos_p * 1000.0;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 39:
          tx_pattern = 39;
          rx_pattern = 79;
          break;
        case 79:
          pos_i= re_val;
          pos_i_ROM = pos_i * 1000.0;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 40:
          tx_pattern = 40;
          rx_pattern = 80;
          break;
        case 80:
          pos_d= re_val;
          pos_d_ROM = pos_d * 1000.0;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 41:
          tx_pattern = 41;
          rx_pattern = 81;
          break;
        case 81:
          Manual_climb_velocity = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 42:
          tx_pattern = 42;
          rx_pattern = 82;
          break;
        case 82:
          Manual_Descend_velocity = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 43:
          tx_pattern = 43;
          rx_pattern = 83;
          break;
        case 83:
          hand_brake_current = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 44:
          tx_pattern = 44;
          rx_pattern = 84;
          break;
        case 84:
          brake_current = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 45:
          tx_pattern = 45;
          rx_pattern = 85;
          break;
        case 85:
          Low_brake_current = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 46:
          tx_pattern = 46;
          rx_pattern = 86;
          break;
        case 86:
          starting_count = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 47:
          tx_pattern = 47;
          rx_pattern = 87;
          break;
        case 87:
          stop_wait = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 48:
          tx_pattern = 48;
          rx_pattern = 88;
          break;
        case 88:
          position_control_velocity = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 49:
          tx_pattern = 49;
          rx_pattern = 89;
          break;
        case 89:
          position_control_down_velocity = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 50:
          tx_pattern = 50;
          rx_pattern = 90;
          break;
        case 90:
          low_speed_dist = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 51:
          tx_pattern = 51;
          rx_pattern = 91;
          break;
        case 91:
          down_margin = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 52:
          tx_pattern = 52;
          rx_pattern = 92;
          break;
        case 92:
          tof_emg_dist = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 53:
          tx_pattern = 53;
          rx_pattern = 93;
          break;
        case 93:
          tof_target_dist = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 54:
          tx_pattern = 54;
          rx_pattern = 94;
          break;
        case 94:
          torque_ODrive = re_val;
          torque_ODrive_ROM = (unsigned int)(torque_ODrive * 1000.0);
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 55:
          tx_pattern = 55;
          rx_pattern = 95;
          break;
        case 95:
          ODrive_offset = re_val;
          ODrive_offset_ROM = (unsigned int)(ODrive_offset * 100.0);
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 56:
          tx_pattern = 56;
          rx_pattern = 96;
          break;
        case 96:
          ODrive_uninstall_distance = re_val;
          ODrive_uninstall_distance_ROM = (unsigned int)(ODrive_uninstall_distance * 100.0);
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 57:
          tx_pattern = 57;
          rx_pattern = 97;
          break;
        case 97:
          throttle_current = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 58:
          tx_pattern = 58;
          rx_pattern = 98;
          break;
        case 98:
          acccel_distance = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 59:
          tx_pattern = 59;
          rx_pattern = 99;
          break;
        case 99:
          start_current = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 61:
          tx_pattern = 61;
          rx_pattern = 101;
          break;
        case 101:
          IDLER_control_flag = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 62:
          tx_pattern = 62;
          rx_pattern = 102;
          break;
        case 102:
          emg_slip_flag = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 63:
          tx_pattern = 63;
          rx_pattern = 103;
          break;
        case 103:
          emg_button_flag = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 64:
          tx_pattern = 64;
          rx_pattern = 104;
          break;
        case 104:
          emg_tof_flag = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;
        
        case 65:
          tx_pattern = 65;
          rx_pattern = 105;
          break;
        case 105:
          current_control_flag = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;

        case 66:
          tx_pattern = 66;
          rx_pattern = 106;
          break;
        case 106:
          IDLER_veL_control_flag = re_val;
          eeprom_write();
          tx_pattern = 1;
          rx_pattern = 0;
          break;



        }
        } else if( xbee_re_buffer[xbee_index] ==  'T' ||  xbee_re_buffer[xbee_index] == 't'){
          rx_pattern  = 0;
          tx_pattern  = 101;
          Serial2.printf("\n");
        }else if( xbee_re_buffer[xbee_index] ==  'S' ||  xbee_re_buffer[xbee_index] == 's'){
          start_flag = true;
          rx_pattern = 0;
          tx_pattern = 1;
          Serial2.printf("\n");
        }else if( xbee_re_buffer[xbee_index] ==  'B' ||  xbee_re_buffer[xbee_index] == 'b'){
          Serial2.printf("\n\n");
          Serial2.printf(" Hand Brake On \n ");
          current_brake_flag = false;
          hand_brake_flag = true;
          rx_pattern  = 0;
          tx_pattern  = 1;
          Serial2.printf("\n");
        }else if( xbee_re_buffer[xbee_index] ==  'H' ||  xbee_re_buffer[xbee_index] == 'h'){
          Serial2.printf("\n\n");
          Serial2.printf(" Current Brake On \n ");
          hand_brake_flag = false;
          current_brake_flag = true;
          rx_pattern  = 0;
          tx_pattern  = 1;
          Serial2.printf("\n");
        }else if( xbee_re_buffer[xbee_index] ==  'R' ||  xbee_re_buffer[xbee_index] == 'r'){
          Serial2.printf("\n\n");
          Serial2.printf(" Brake releace \n ");
          hand_brake_flag = false;
          current_brake_flag = false;
          rx_pattern  = 0;
          tx_pattern  = 1;
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'U' ||  xbee_re_buffer[xbee_index] == 'u') && pattern != 81){
          Serial2.printf("\n\n\n\n");
          Serial2.printf(" Manual_Climb S : Start \n ");
          pattern = 61;
          rx_pattern  = 0;
          tx_pattern  = 1;
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'D' ||  xbee_re_buffer[xbee_index] == 'd') && pattern != 81){
          Serial2.printf("\n\n\n\n");
          Serial2.printf(" Manual_Descend S : Start \n ");
          pattern = 71;
          rx_pattern  = 0;
          tx_pattern  = 1;
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'O' ||  xbee_re_buffer[xbee_index] == 'o') && (pattern == 1 || pattern == 81)){
          Serial2.printf("\n\n\n\n");
          Serial2.printf(" Manual ODrive U : 10UP D : 10DOWN \n ");
          Serial2.printf("               I : 1 UP F : 1 DOWN \n ");
          Stamp_id = POSITION_CONTROL_TRAJ; 
          pattern = 81;
          rx_pattern  = 0;
          tx_pattern  = 1;
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'C' ||  xbee_re_buffer[xbee_index] == 'c') && pattern == 1){
          Serial2.printf("\n\n\n\n");
          Serial2.printf(" FULL_CALIBRATION_SEQUENCE \n ");
          Stamp_id = ODRIVE_CALIBRATION; 
          rx_pattern  = 0;
          tx_pattern  = 1;
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'L' ||  xbee_re_buffer[xbee_index] == 'l')){
          Serial2.printf("\n\n\n\n");
          Serial2.printf(" CLOSED_LOOP_CONTROL \n ");
          Stamp_id = ODRIVE_CLOSED_LOOP;
          rx_pattern  = 0;
          tx_pattern  = 1;
          Serial2.printf("\n");
        }else if( xbee_re_buffer[xbee_index] ==  'V' ||  xbee_re_buffer[xbee_index] == 'v'){
          Serial2.printf("\n\n\n\n");
          Serial2.printf(" ODrive Reboot \n ");
          Stamp_id = ODRIVE_REBOOT; 
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'U' ||  xbee_re_buffer[xbee_index] == 'u') && pattern == 81 && Current_State_F == 8){
          Serial2.printf("\n\n\n\n");
          Serial2.printf(" 10UP \n ");
          Stamp_id = UP10; 
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'I' ||  xbee_re_buffer[xbee_index] == 'i') && pattern == 81 && Current_State_F == 8){
          Serial2.printf("\n\n\n\n");
          Serial2.printf(" 1UP \n ");
          Stamp_id = UP1;
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'D' ||  xbee_re_buffer[xbee_index] == 'd') && pattern == 81 && Current_State_F == 8){
          Serial2.printf("\n\n\n\n");
          Serial2.printf(" 10DONW \n ");
          Stamp_id = DOWN10;
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'F' ||  xbee_re_buffer[xbee_index] == 'f') && pattern == 81 && Current_State_F == 8){
          Serial2.printf("\n\n\n\n");
          Serial2.printf(" 1Down \n ");
          Stamp_id = DOWN1;
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'A' ||  xbee_re_buffer[xbee_index] == 'a') && pattern == 81 && Current_State_F == 8){
          Serial2.printf("\n\n\n\n");
          Serial2.printf(" uninstall \n ");
          send_data[3] = (int)(ODrive_uninstall_distance * 100.0) & 0xff;
          send_data[4] = (int)(ODrive_uninstall_distance * 100.0) >> 8 & 0xff;
          Stamp_id = UNINSTALL; 
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'X' ||  xbee_re_buffer[xbee_index] == 'x') && pattern == 81 && Current_State_F == 8){
          Serial2.printf("\n");
          Serial2.printf(" OFFSET \n ");
          Stamp_id = ODRIVE_OFFSET; 
          send_data[3] = (int)(ODrive_offset * 1000.0) & 0xff;
          send_data[4] = (int)(ODrive_offset * 1000.0) >> 8 & 0xff;          
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'Z' ||  xbee_re_buffer[xbee_index] == 'z') && Current_State_F == 8 && pattern == 1){
          send_data[3] = (int)(torque_ODrive * 1000.0) & 0xff;
          send_data[4] = (int)(torque_ODrive * 1000.0) >> 8 & 0xff;
          Stamp_id = TORQUE_CONTROL; 
          rx_pattern = 0;
          tx_pattern = 1;
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'M' ||  xbee_re_buffer[xbee_index] == 'm') && (pattern == 1 || pattern == 6) && Current_State_F == 8){
          Serial2.printf("\n\n\n\n");
          Serial2.printf(" ODrive_start_possition Set \n ");
          Stamp_id = CALIBRATION; 
          arm_start_pos = arm_height;
          pattern = 1;
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'N' ||  xbee_re_buffer[xbee_index] == 'n') && pattern == 1 && Current_State_F == 8){
          Serial2.printf("\n\n\n\n");
          Serial2.printf(" ODrive_Position_Control ON \n ");
          Stamp_id = ARM_CONTROL; 
          can_time_buff = millis();
          Serial2.printf("\n");
        }else if( xbee_re_buffer[xbee_index] ==  'Q' ||  xbee_re_buffer[xbee_index] == 'q'){
          reset_flag = true;
          rx_pattern = 0;
          tx_pattern = 1;
          Serial2.printf("\n");
        }else if(( xbee_re_buffer[xbee_index] ==  'K' ||  xbee_re_buffer[xbee_index] == 'k') && pattern == 0){
          trajectory_control_flag = false;
          position_control_flag = false;
          rx_pattern = 0;
          tx_pattern = 1;
          delay(10);
          pattern = 1;
          Serial2.printf("\n");
        }else if( xbee_re_buffer[xbee_index] ==  ' '){
          Serial2.printf("\n\n");
          Serial2.printf(" Emergency Stop Enable Spacekey\n ");
          time_buff = millis();
          Stamp_id = EMG_STOP; 
          trajectory_control_flag = false;
          position_control_flag = false;
          velocity_control_flag = false;
          hand_brake_flag = false;
          current_brake_flag = true;
          start_flag = false;
          operation_flag = false;
          Encoder_check_flag = false; 
          time_buff = millis();
          pattern = 200;
          rx_pattern = 0;
          Serial2.printf("\n");
        }else{
            xbee_index++;
        }
  }
}

//XBee_TX
//------------------------------------------------------------------//
void xbee_se(void){

    switch (tx_pattern){
      //Waiting Command
      case 0:
        break;

      case 1:
        Serial2.printf("\n\n\n\n\n\n");
        Serial2.printf("climber Controller (M5Stack version) \n");

        Serial2.printf(" 31 :  Climb Height             [%4d]\n",climb_height);
        Serial2.printf(" 32 :  Climb Velocity           [%4d]\n",climb_velocity); 
        Serial2.printf(" 33 :  Climber Accel            [%4.2f]\n",climber_accel);
        Serial2.printf(" 34 :  Climber Decel            [%4.2f]\n",climber_decel);
        Serial2.printf(" 35 :  Down Velocity            [%4d]\n",down_velocity);
        Serial2.printf(" 36 :  Down Accel               [%4.2f]\n",down_accel);
        Serial2.printf(" 37 :  Down Decel               [%4.2f]\n",down_decel);
        Serial2.printf(" 38 :  Pos Gain P               [%4.3f]\n",pos_p);
        Serial2.printf(" 39 :  Pos Gain I               [%4.3f]\n",pos_i);
        Serial2.printf(" 40 :  Pos Gain D               [%4.3f]\n",pos_d);
        Serial2.printf(" 41 :  Manual Climb Velocity    [%4d]\n",Manual_climb_velocity);
        Serial2.printf(" 42 :  Manual Descend Velocity  [%4d]\n",Manual_Descend_velocity);
        Serial2.printf(" 43 :  Hand Brake Current       [%4d]\n",hand_brake_current);
        Serial2.printf(" 44 :  Motor Brake Current      [%4d]\n",brake_current);
        Serial2.printf(" 45 :  Motor Low Brake Current  [%4d]\n",Low_brake_current);
        Serial2.printf(" 46 :  Starting Count           [%4d]\n",starting_count);
        Serial2.printf(" 47 :  Stop Wait                [%4d]\n",stop_wait);
        Serial2.printf(" 48 :  Position Control Velocity[%4d]\n",position_control_velocity);
        Serial2.printf(" 49 :  Position Down Velocity   [%4d]\n",position_control_down_velocity); 
        Serial2.printf(" 50 :  Low Speed Distance       [%4d]\n",low_speed_dist);
        Serial2.printf(" 51 :  Down Margin              [%4d]\n",down_margin);
        Serial2.printf(" 52 :  ToF EMG Distance         [%4d]\n",tof_emg_dist);
        Serial2.printf(" 53 :  ToF Target Distance      [%4d]\n",tof_target_dist);
        Serial2.printf(" 54 :  TORQUE ODrive            [%4.2f]\n",torque_ODrive);
        Serial2.printf(" 55 :  ODrive Offset            [%4.2f]\n",ODrive_offset);
        Serial2.printf(" 56 :  ODrive Uninstall Distance[%4.2f]\n",ODrive_uninstall_distance);
        Serial2.printf(" 57 :  Full Throttle Current    [%4d]\n",throttle_current);
        Serial2.printf(" 58 :  Accel Distance           [%4d]\n",acccel_distance);
        Serial2.printf(" 59 :  start_current           [%4d]\n",start_current);


        Serial2.printf("\n");
        Serial2.printf(" T : Terementry\n");
        Serial2.printf(" S : Start_Climb\n");
        Serial2.printf(" U : Manual  Climb\n");
        Serial2.printf(" D : Descend Manual\n");
        Serial2.printf(" Q : Reset\n");
        Serial2.printf(" B : Hand brake On\n");
        Serial2.printf(" H : Current_brake On\n");
        Serial2.printf(" R : brake releace\n");
        Serial2.printf(" K : Skip LCD Mode\n");

        Serial2.printf("\n");

        tx_pattern = 0;
      break;
      
    case 2:
        Serial2.printf("\n\n\n\n\n\n");
        Serial2.printf("climber Controller (M5Stack version) \n");
        
        Serial2.printf(" 61 :  IDLER Control Flag       [%4d]\n",IDLER_control_flag);
        Serial2.printf(" 62 :  EMG Slip flag            [%4d]\n",emg_slip_flag); 
        Serial2.printf(" 63 :  Emgbutton check flag     [%4d]\n",emg_button_flag);
        Serial2.printf(" 64 :  EMG ToF Flag             [%4d]\n",emg_tof_flag);
        Serial2.printf(" 65 :  Current Control Flag     [%4d]\n",current_control_flag);
        Serial2.printf(" 66 :  IDLER Vel Control Flag   [%4d]\n",IDLER_veL_control_flag);
        

        Serial2.printf("\n");
        Serial2.printf(" T : Terementry\n");
        Serial2.printf(" S : Start_Climb\n");
        Serial2.printf(" U : Manual  Climb\n");
        Serial2.printf(" D : Descend Manual\n");
        Serial2.printf(" Q : Reset\n");
        Serial2.printf(" B : Hand brake On\n");
        Serial2.printf(" H : Current_brake On\n");
        Serial2.printf(" R : brake releace\n");
  
        Serial2.printf("\n");
  
        tx_pattern = 0;
      break;
    
    case 3:
        Serial2.printf("\n\n\n\n\n\n");
        Serial2.printf("climber Controller (M5Stack version) \n");
        
        Serial2.printf(" 51 :  Climb Height             [%4d]\n",climb_height);
        Serial2.printf(" 52 :  Climb Velocity           [%4d]\n",climb_velocity); 
        Serial2.printf(" 53 :  Climber Accel            [%4.2f]\n",climber_accel);
        Serial2.printf(" 54 :  Climber Decel            [%4.2f]\n",climber_decel);
        Serial2.printf(" 55 :  Down Velocity            [%4d]\n",down_velocity);
        Serial2.printf(" 56 :  Down Accel               [%4.2f]\n",down_accel);
        Serial2.printf(" 57 :  Down Decel               [%4.2f]\n",down_decel);
        Serial2.printf(" 58 :  Pos Gain P               [%4.3f]\n",pos_p);
        Serial2.printf(" 59 :  Pos Gain I               [%4.3f]\n",pos_i);
        Serial2.printf(" 60 :  Pos Gain D               [%4.3f]\n",pos_d);
        Serial2.printf(" 61 :  Manual Climb Velocity    [%4d]\n",Manual_climb_velocity);
        Serial2.printf(" 62 :  Manual Descend Velocity  [%4d]\n",Manual_Descend_velocity);
        Serial2.printf(" 63 :  Hand Brake Current       [%4d]\n",hand_brake_current);
        Serial2.printf(" 64 :  Motor Brake Current      [%4d]\n",brake_current);
        Serial2.printf(" 65 :  Motor Low Brake Current  [%4d]\n",Low_brake_current);
        Serial2.printf(" 66 :  Starting Count           [%4d]\n",starting_count);
        Serial2.printf(" 67 :  Stop Wait                [%4d]\n",stop_wait);
  
        Serial2.printf("\n");
        Serial2.printf(" T : Terementry\n");
        Serial2.printf(" S : Start_Climb\n");
        Serial2.printf(" U : 10UP\n");
        Serial2.printf(" I : 1UP\n");
        Serial2.printf(" D : 10DOWN\n");
        Serial2.printf(" F : 1DOWN\n");
        Serial2.printf(" Q : Reset\n");
        Serial2.printf(" B : Hand brake On\n");
        Serial2.printf(" H : Current_brake On\n");
        Serial2.printf(" R : brake releace\n");
  
        Serial2.printf("\n");
  
        tx_pattern = 0;
      break;

    case 31:
       Serial2.printf(" Climb Height [%3d]\n " ,climb_height);
       Serial2.printf(" Please enter 0 to 1000 -> ");
       tx_pattern = 5;
       break;
      
    case 32:
        Serial2.printf(" Climb Velocity [%4d]\n ",climb_velocity);
        Serial2.printf(" Please enter 0 to 200 -> ");
        tx_pattern = 5;
        break;

    case 33:
        Serial2.printf(" Climber Accel    [%4.2f]\n",climber_accel);
        Serial2.printf(" Please enter 0 to 25 -> ");
        tx_pattern = 5;
        break;

    case 34:
        Serial2.printf(" Climber Decel [%4.2f]\n ",climber_decel);
        Serial2.printf(" Please enter 0 to 25 -> ");
        tx_pattern = 5;
        break;

    case 35:
        Serial2.printf(" downvelocity  [%4d]\n",down_velocity);
        Serial2.printf(" Please enter 0 to 200 -> ");
        tx_pattern = 5;
        break;

    case 36:
        Serial2.printf(" downaccel         [%4.2f]\n",down_accel);
        Serial2.printf(" Please enter 0 to 25 -> ");
        tx_pattern = 5;
        break;

    case 37:
        Serial2.printf(" downdecel        [%4.2f]\n",down_decel);
        Serial2.printf(" Please enter 0 to 25 -> ");
        tx_pattern = 5;
        break;
    
    case 38:
        Serial2.printf(" Pos Gain P       [%4.3f]\n",pos_p);
        Serial2.printf(" Please enter 0 to 65 -> ");
        tx_pattern = 5;
        break;

    case 39:
        Serial2.printf(" Pos Gain I        [%4.3f]\n",pos_i);
        Serial2.printf(" Please enter 0 to 65 -> ");
        tx_pattern = 5;
        break;

    case 40:
        Serial2.printf(" Pos Gain D        [%4.3f]\n",pos_d);
        Serial2.printf(" Please enter 0 to 65 -> ");
        tx_pattern = 5;
        break;

    case 41:
        Serial2.printf(" Manual_climb_velocity [%4d]\n ",Manual_climb_velocity);
        Serial2.printf(" Please enter 0 to 100 -> ");
        tx_pattern = 5;
        break;

    case 42:
        Serial2.printf(" Manual_Descend_velocity [%4d]\n ",Manual_Descend_velocity);
        Serial2.printf(" Please enter 0 to 100 -> ");
        tx_pattern = 5;
    
    case 43:
        Serial2.printf(" Hand Brake Current [%4d]\n",hand_brake_current);
        Serial2.printf(" Please enter 0 to 100 -> ");
        tx_pattern = 5;
        break;

    case 44:
        Serial2.printf(" Brake Current [%4d]\n",brake_current);
        Serial2.printf(" Please enter 0 to 100 -> ");
        tx_pattern = 5;
        break;

    case 45:
        Serial2.printf(" Motor Low Brake Current[%4d]\n",Low_brake_current);
        Serial2.printf(" Please enter 0 to 100 -> ");
        tx_pattern = 5;
        break;
    
    case 46:
        Serial2.printf(" Starting Count [%2d]\n ",starting_count);
        Serial2.printf(" Please enter 0 to 60 -> ");
        tx_pattern = 5;
        break;

    case 47:
        Serial2.printf(" Stop Wait [%2d]\n ",stop_wait);
        Serial2.printf(" Please enter 0 to 60 -> ");
        tx_pattern = 5;
        break;
    
    case 48:
        Serial2.printf(" Position_control Velocity [%2d]\n ",position_control_velocity);
        Serial2.printf(" Please enter 0 to 10 -> ");
        tx_pattern = 5;
        break;
      
    case 49:
        Serial2.printf(" Position_control Down Velocity [%2d]\n ",position_control_down_velocity);
        Serial2.printf(" Please enter 0 to 10 -> ");
        tx_pattern = 5;
        break;
    
    case 50:
        Serial2.printf(" Lown Speed Distance [%2d]\n ",low_speed_dist);
        Serial2.printf(" Please enter 0 to 255 -> ");
        tx_pattern = 5;
        break;
      
    case 51:
        Serial2.printf(" Down Margin [%2d]\n ",down_margin);
        Serial2.printf(" Please enter 0 to 255 -> ");
        tx_pattern = 5;
        break;
    
    case 52:
        Serial2.printf(" ToF EMG Distance [%4d][cm]\n ",tof_emg_dist);
        Serial2.printf(" Please enter 0 to 500 -> ");
        tx_pattern = 5;
        break;

    case 53:
        Serial2.printf(" ToF Target Distance [%4d][cm]\n ",tof_target_dist);
        Serial2.printf(" Please enter 0 to 500 -> ");
        tx_pattern = 5;
        break;
    
    case 54:
        Serial2.printf(" TORQUE ODrive [%4.2f]\n ",torque_ODrive);
        Serial2.printf(" Please enter 0 to 10 -> ");
        tx_pattern = 5;
        break;
    
    case 55:
        Serial2.printf(" ODrive Offset [%4.2f]\n ",ODrive_offset);
        Serial2.printf(" Please enter 0 to 655 -> ");
        tx_pattern = 5;
        break;
    
    case 56:
        Serial2.printf(" ODrive Uninstall Distance [%4.2f]\n ",ODrive_uninstall_distance);
        Serial2.printf(" Please enter 0 to 20 -> ");
        tx_pattern = 5;
        break;
      
    case 57:
        Serial2.printf(" Full throttle Current [%4d]\n ",throttle_current);
        Serial2.printf(" Please enter 0 to 95 -> ");
        tx_pattern = 5;
        break;
    
    case 58:
        Serial2.printf(" Accel Distance [%4d]\n ",acccel_distance);
        Serial2.printf(" Please enter 0 to 5 -> ");
        tx_pattern = 5;
        break;
    
    case 59:
        Serial2.printf(" start_current [%4d]\n ",start_current);
        Serial2.printf(" Please enter 0 to 95 -> ");
        tx_pattern = 5;
        break;

    case 61:
        Serial2.printf(" IDLER Control Flag [%4d]\n ",IDLER_control_flag);
        Serial2.printf(" Please enter 0 or 1 -> ");
        tx_pattern = 5;
        break;
    case 62:
        Serial2.printf(" EMG Slip Flag [%4d]\n ",emg_slip_flag);
        Serial2.printf(" Please enter 0 or 1 -> ");
        tx_pattern = 5;
        break;
    case 63:
        Serial2.printf(" EMG Button Flag [%4d]\n ",emg_button_flag);
        Serial2.printf(" Please enter 0 or 1 -> ");
        tx_pattern = 5;
        break;
    case 64:
        Serial2.printf(" EMG ToF Flag [%4d]\n ",emg_tof_flag);
        Serial2.printf(" Please enter 0 or 1 -> ");
        tx_pattern = 5;
        break;
    case 65:
        Serial2.printf(" Current Control Flag [%4d]\n ",current_control_flag);
        Serial2.printf(" Please enter 0 or 1 -> ");
        tx_pattern = 5;
        break;
    case 66:
        Serial2.printf(" IDLER Vel Control Flag [%4d]\n ",IDLER_veL_control_flag);
        Serial2.printf(" Please enter 0 or 1 -> ");
        tx_pattern = 5;
        break;

    //Waiting value
    case 5:
      break;

    //Telementry Mode
    case 101:
      break;

    
  }
}

// EEPROM Write
//------------------------------------------------------------------//
void eeprom_write(void){
    EEPROM.write(0,  (climb_height & 0xFF));
    EEPROM.write(1,  (climb_height>>8 & 0xFF));
    EEPROM.write(2,  (climb_velocity & 0xFF));
    EEPROM.write(3,  (climber_accel_ROM & 0xFF));
    EEPROM.write(4,  (climber_accel_ROM>>8 & 0xFF));
    EEPROM.write(5,  (climber_decel_ROM & 0xFF));
    EEPROM.write(6,  (climber_decel_ROM>>8 & 0xFF));
    EEPROM.write(7,  (down_velocity & 0xFF));
    EEPROM.write(8,  (down_accel_ROM & 0xFF));
    EEPROM.write(9,  (down_accel_ROM>>8 & 0xFF));
    EEPROM.write(10, (down_decel_ROM & 0xFF));
    EEPROM.write(11, (down_decel_ROM>>8 & 0xFF));
    EEPROM.write(12, (pos_p_ROM & 0xFF));
    EEPROM.write(13, (pos_p_ROM >>8 & 0xFF));
    EEPROM.write(14, (pos_i_ROM & 0xFF));
    EEPROM.write(15, (pos_i_ROM >>8 & 0xFF));
    EEPROM.write(16, (pos_d_ROM & 0xFF));
    EEPROM.write(17, (pos_d_ROM >>8 & 0xFF));
    EEPROM.write(18, (hand_brake_current));
    EEPROM.write(19, (brake_current));
    EEPROM.write(20, (Low_brake_current));
    EEPROM.write(21, (starting_count));
    EEPROM.write(22, (stop_wait));
    EEPROM.write(23, (position_control_velocity));
    EEPROM.write(24, (position_control_down_velocity));
    EEPROM.write(25, (low_speed_dist));
    EEPROM.write(26, (down_margin));
    EEPROM.write(27, (tof_emg_dist & 0xFF));
    EEPROM.write(28, (tof_emg_dist>>8 & 0xFF));
    EEPROM.write(29, (tof_target_dist & 0xFF));
    EEPROM.write(30, (tof_target_dist>>8 & 0xFF));
    EEPROM.write(31, (torque_ODrive_ROM & 0xFF));
    EEPROM.write(32, (torque_ODrive_ROM>>8 & 0xFF));
    EEPROM.write(33, (IDLER_control_flag));
    EEPROM.write(34, (emg_slip_flag));
    EEPROM.write(35, (emg_button_flag));
    EEPROM.write(36, (emg_tof_flag));
    EEPROM.write(37, (current_control_flag));
    EEPROM.write(38, (ODrive_offset_ROM & 0xFF));
    EEPROM.write(39, (ODrive_offset_ROM>>8 & 0xFF));
    EEPROM.write(40, (Manual_climb_velocity & 0xFF));
    EEPROM.write(41, (Manual_Descend_velocity & 0xFF));
    EEPROM.write(42, (ODrive_uninstall_distance_ROM & 0xFF));
    EEPROM.write(43, (ODrive_uninstall_distance_ROM>>8 & 0xFF));
    EEPROM.write(44, (throttle_current & 0xFF));
    EEPROM.write(45, (acccel_distance & 0xFF));
    EEPROM.write(46, (start_current & 0xFF));
    EEPROM.write(47, (IDLER_veL_control_flag & 0xFF));
    delay(10);
    EEPROM.commit();
    delay(10);
}

//EEPROM read
//------------------------------------------------------------------//
void eeprom_read(void){
    climb_height = EEPROM.read(0) + (EEPROM.read(1)<<8);
    climb_velocity = EEPROM.read(2);
    climber_accel_ROM = EEPROM.read(3) + (EEPROM.read(4)<<8);
    climber_accel = (float)climber_accel_ROM / 10.0;
    climber_decel_ROM = EEPROM.read(5) + (EEPROM.read(6)<<8);
    climber_decel = (float)climber_decel_ROM / 10.0;
    down_velocity = EEPROM.read(7);
    down_accel_ROM = EEPROM.read(8) + (EEPROM.read(9)<<8);
    down_accel = (float)down_accel_ROM / 10.0;
    down_decel_ROM = EEPROM.read(10) + (EEPROM.read(11)<<8);
    down_decel = (float)down_decel_ROM / 10.0;
    pos_p_ROM = EEPROM.read(12) + (EEPROM.read(13)<<8);
    pos_p = (float)(pos_p_ROM) / 1000.0;
    pos_i_ROM = EEPROM.read(14) + (EEPROM.read(15)<<10);
    pos_i = (float)(pos_i_ROM) / 1000.0;
    pos_d_ROM = EEPROM.read(16) + (EEPROM.read(17)<<8);
    pos_d = (float)(pos_d_ROM) / 1000.0;
    hand_brake_current = EEPROM.read(18);
    brake_current = EEPROM.read(19);
    Low_brake_current = EEPROM.read(20);
    starting_count = EEPROM.read(21);
    stop_wait = EEPROM.read(22);
    position_control_velocity = EEPROM.read(23);
    position_control_down_velocity = EEPROM.read(24);
    low_speed_dist = EEPROM.read(25);
    down_margin = EEPROM.read(26);
    tof_emg_dist = EEPROM.read(27) + (EEPROM.read(28)<<8);
    tof_target_dist = EEPROM.read(29) + (EEPROM.read(30)<<8);
    torque_ODrive_ROM = EEPROM.read(31) + (EEPROM.read(32)<<8);
    torque_ODrive = (float)(torque_ODrive_ROM) / 1000.0;
    IDLER_control_flag = EEPROM.read(33);
    emg_slip_flag = EEPROM.read(34);
    emg_button_flag = EEPROM.read(35);
    emg_tof_flag = EEPROM.read(36);
    current_control_flag = EEPROM.read(37);
    ODrive_offset_ROM = EEPROM.read(38) + (EEPROM.read(39)<<8);
    ODrive_offset = (float)(ODrive_offset_ROM) / 100.0;
    Manual_climb_velocity = EEPROM.read(40);
    Manual_Descend_velocity = EEPROM.read(41);
    ODrive_uninstall_distance_ROM = EEPROM.read(42) + (EEPROM.read(43)<<8);
    ODrive_uninstall_distance = (float)(ODrive_uninstall_distance_ROM) / 100.0;
    throttle_current = EEPROM.read(44);
    acccel_distance = EEPROM.read(45);
    start_current = EEPROM.read(46);
    IDLER_veL_control_flag = EEPROM.read(47);
    delay(10);
}
