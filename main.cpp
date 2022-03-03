#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <curses.h>
#include "vive.h"

#define frequency  25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C

#define HB_LIM 100 // heartbeat counter threshold

//add constants
#define PWM_MAX          1900 // 1800
#define NEUTRAL_PWR      1450 // 1435
#define frequency  25000000.0
#define LED0              0x6			
#define LED0_ON_L         0x6		
#define LED0_ON_H         0x7		
#define LED0_OFF_L        0x8		
#define LED0_OFF_H        0x9		
#define LED_MULTIPLYER      4	

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};
 
enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};
 
int setup_imu();
void calibrate_imu();      
void read_imu();    
void update_filter();
void setup_keyboard();
void comp_filter();
void safety_check();
void trap(int signal);
void motor_pwm();
void set_PWM( uint8_t channel, float time_on_us);
void init_pwm();
void init_motor(uint8_t channel);

//global variables
int imu;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //gyro xyz, accel xyz
long time_curr;
long time_prev;
long time_curr_vive;
long time_prev_vive;

struct timespec te ,ve;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
float roll_position = 0; // self add
float pitch_position = 0;// self add
float roll_gyro = 0;// self add
float pitch_gyro = 0;// self add
float yaw_gyro =0;
int prev_hb = 0;
int pwm;
bool pause = true;

float des_pitch_joy;
float des_roll_joy;
float des_yaw_joy;

float des_pitch;
float des_roll;
float des_yaw;

float des_pitch_vive;
float des_roll_vive;
float des_yaw_vive;


float roll_gyro_delta;
float pitch_gyro_delta;
float yaw_gyro_delta;
float pitch_I_term;
float roll_I_term;

int neutral_thrust = 1400;
int thrust_gain = 150;
int thruster;

int motor0_pwm;
int motor1_pwm;
int motor2_pwm;
int motor3_pwm;

//get rid of later
int P_button =0;
int D_button =0;



Position local_p, prev_p, desired_p;

struct Keyboard {
  int keypress;
  int pitch;
  int roll;
  int yaw;
  int thrust;
  int sequence_num;
};
// };
// struct Keyboard {
//   int keypress; //
//   int pitch;
//   int roll;
//   int yaw;
//   int thrust;
//   int sequence_num;
// };

Keyboard* shared_memory;
int run_program=1;

// FILE *file_roll = fopen("roll_position.csv", "w");
FILE *plot = fopen("plot.csv", "w");

int main (int argc, char *argv[])
{
    //in main function before calibrate imu add
    init_shared_memory();
    init_pwm();
    init_motor(0);
    init_motor(1);
    init_motor(2);
    init_motor(3);
    delay(1000);
    setup_imu();
    calibrate_imu();
    setup_keyboard();
    signal(SIGINT, &trap);
    
    while(run_program==1)
    {
      safety_check();
      read_imu();      
      update_filter();   
      motor_pwm();
      local_p=*position;  
      
      // printf("\nwe rolling"); 
    }
    return 0;
}

void motor_pwm(){
  Keyboard keyboard=*shared_memory;


  static int version_counter;

  
  

  if(prev_p.version == local_p.version){
    version_counter++;
    if(version_counter > 100){
      run_program=0;
    }
  }else{
    version_counter = 0;
  }



  
  // gains
  int P_pitch = 7   ; //10 //15
  int D_pitch = 50; // 80  //690 // 80
  static float I_pitch = 0.03; // 0.03 // 0.042069

  // P_pitch = P_pitch + P_button;
  // D_pitch = D_pitch + D_button;


  int P_roll = 6; // 10
  int D_roll = 65; // 150 // 90
  float I_roll = 0.042069; // 0.042069

  int control_yaw = 150; // 10
  float P_yaw = 3; // 150 // 300
  // float I_yaw = 0.0; // 0.042069

  //vive control
  int yaw_vive_scale = 50;
  float P_pitch_vive = 0.03;
  float D_pitch_vive = 0.1 ;
  float P_roll_vive = 0.03;//maybe up
  float D_roll_vive = 0.1;

  float x_err = -(local_p.x - desired_p.x);
  float y_err = (local_p.y - desired_p.y);
  
  // float x_delta_vive = local_p.x - prev_p.x;
  // float y_delta_vive = local_p.y - prev_p.y;

  if(prev_p.version != local_p.version){
      // printf("\n\r desired: x=%5.2f, y=%5.2f, z=%5.2f, yaw=%5.2f",desired_p.x, desired_p.y,desired_p.z,desired_p.yaw);
      timespec_get(&ve,TIME_UTC);
      time_curr_vive=ve.tv_nsec;
      //compute time since last execution
      float vive_diff=time_curr_vive-time_prev_vive;           
      
      //check for rollover
      if(vive_diff<=0)
      {
        vive_diff+=1000000000;
      }
      //convert to seconds
      vive_diff=vive_diff/1000000000;
      time_prev_vive=time_curr_vive;

      float x_delta_vive = -(local_p.x - prev_p.x);//*vive_diff;
      float y_delta_vive = (local_p.y - prev_p.y);//*vive_diff;




      des_pitch_vive =  y_err*P_pitch_vive + y_delta_vive * D_pitch_vive ; //

      des_roll_vive =  x_err*P_roll_vive + x_delta_vive * D_roll_vive ; // 




      if(local_p.x > 1200 ||local_p.y > 1200 ||local_p.x < -1200 ||local_p.y < -1200 ){
        printf("\n\rGET BACK TO THE BATTLEFIELD!");
        // run_program=0;
      }
      // printf("current: x=%5.2f, y=%5.2f, z=%5.2f, yaw=%5.2f",current_p.x, current_p.y,current_p.z,current_p.yaw);
    }
  




  pitch_I_term += (des_pitch_joy+pitch_position)*I_pitch;

  if(pitch_I_term > 150){
    pitch_I_term = 150;
  }
  if(pitch_I_term < -150){
    pitch_I_term = -150;
  }

  roll_I_term += (des_roll_joy+roll_position)*I_roll;

  if(roll_I_term > 150){
    roll_I_term = 150;
  }
  if(roll_I_term < -150){
    roll_I_term = -150;
  }
  // printf("\npitch I_pitch term %f : pitch_position is %f",pitch_I_term,pitch_position);

  //thrust scaling
  thruster = (keyboard.thrust/128.0)*160+NEUTRAL_PWR; 
  // printf("\nthuster = %d, keyboard thrust = %d",thruster,keyboard.thrust);

  des_pitch_joy = ((keyboard.pitch-128.0)/128.0)*10;  // might need to fix this
  des_roll_joy = ((keyboard.roll-128.0)/128.0)*10;
  des_yaw_joy = ((keyboard.yaw-128.0)/128.0)*control_yaw;



  des_yaw = (local_p.yaw-desired_p.yaw)*yaw_vive_scale;

  des_pitch = des_pitch_joy*0.5 + des_pitch_vive*0.5;
  des_roll = des_roll_joy *0.5 + des_roll_vive*0.5;
  
  

  // thruster = (keyboard.thrust - 0) * (1800-1000)/(255-0)+1000;
  // thruster = ((keyboard.thrust/255)*1800) + 1000
  printf("\n ddesired = %5.2f, vive=%5.2f, current = %5.2f, I_Val=%5.2f, P=%d , D=%d, I=%5.2f  ",des_pitch,des_pitch_vive, pitch_position, pitch_I_term,P_pitch,D_pitch,I_pitch );
  // printf("\n desired = %5.2f, current = %5.2f, I_Val=%5.2f, P=%d , D=%d, I=%5.2f  ",des_roll_joy, roll_position, roll_I_term,P_roll,D_roll,I_roll );
  printf("\n motor0 = %d, motor1 = %d, motor2 = %d, motor3 = %d, yaw = %5.2f, forward=%d, back=%d, left=%d, right=%d",motor0_pwm,motor1_pwm, motor2_pwm, motor3_pwm, yaw_gyro_delta,motor3_pwm+motor0_pwm,motor1_pwm+motor2_pwm,motor2_pwm+motor3_pwm,motor1_pwm+motor0_pwm);



  // motor1_pwm = thruster + pitch_gyro_delta*D_pitch + (des_pitch_joy+pitch_position)*P_pitch + pitch_I_term - roll_gyro_delta*D_roll - (des_roll_joy+roll_position)*P_roll - roll_I_term - (des_yaw_joy-yaw_gyro_delta)*P_yaw; // back right 1
  // motor2_pwm = thruster + pitch_gyro_delta*D_pitch + (des_pitch_joy+pitch_position)*P_pitch + pitch_I_term + roll_gyro_delta*D_roll + (des_roll_joy+roll_position)*P_roll + roll_I_term + (des_yaw_joy-yaw_gyro_delta)*P_yaw; // back left 2 
  // motor3_pwm = thruster - pitch_gyro_delta*D_pitch - (des_pitch_joy+pitch_position)*P_pitch - pitch_I_term + roll_gyro_delta*D_roll + (des_roll_joy+roll_position)*P_roll + roll_I_term - (des_yaw_joy-yaw_gyro_delta)*P_yaw ; // front left 3 
  // motor0_pwm = thruster - pitch_gyro_delta*D_pitch - (des_pitch_joy+pitch_position)*P_pitch - pitch_I_term - roll_gyro_delta*D_roll - (des_roll_joy+roll_position)*P_roll - roll_I_term + (des_yaw_joy-yaw_gyro_delta)*P_yaw ; // front right 0

  motor1_pwm = thruster + pitch_gyro_delta*D_pitch + (des_pitch+pitch_position)*P_pitch + pitch_I_term - roll_gyro_delta*D_roll - (des_roll+roll_position)*P_roll - roll_I_term - (des_yaw-yaw_gyro_delta)*P_yaw; // back right 1
  motor2_pwm = thruster + pitch_gyro_delta*D_pitch + (des_pitch+pitch_position)*P_pitch + pitch_I_term + roll_gyro_delta*D_roll + (des_roll+roll_position)*P_roll + roll_I_term + (des_yaw-yaw_gyro_delta)*P_yaw; // back left 2 
  motor3_pwm = thruster - pitch_gyro_delta*D_pitch - (des_pitch+pitch_position)*P_pitch - pitch_I_term + roll_gyro_delta*D_roll + (des_roll+roll_position)*P_roll + roll_I_term - (des_yaw-yaw_gyro_delta)*P_yaw ; // front left 3 
  motor0_pwm = thruster - pitch_gyro_delta*D_pitch - (des_pitch+pitch_position)*P_pitch - pitch_I_term - roll_gyro_delta*D_roll - (des_roll+roll_position)*P_roll - roll_I_term + (des_yaw-yaw_gyro_delta)*P_yaw ; // front right 0

  // set PWM max value
  if(motor0_pwm > PWM_MAX){
    motor0_pwm = PWM_MAX;
  }
  if(motor1_pwm > PWM_MAX){
    motor1_pwm = PWM_MAX;
  }
  if(motor2_pwm > PWM_MAX){
    motor2_pwm = PWM_MAX;
  }
  if(motor3_pwm > PWM_MAX){
    motor3_pwm = PWM_MAX;
  }

  // set PWM min value
  if(motor0_pwm < 1000){
    motor0_pwm = 1000;
  }
  if(motor1_pwm < 1000){
    motor1_pwm = 1000;
  }
  if(motor2_pwm < 1000){
    motor2_pwm = 1000;
  }
  if(motor3_pwm < 1000){
    motor3_pwm = 1000;
  }

  // send PWM to motors
  if(pause == false){
    set_PWM(1,motor1_pwm);
    set_PWM(2,motor2_pwm);
    set_PWM(3,motor3_pwm);
    set_PWM(0,motor0_pwm);
  } else{
    set_PWM(1,1000);
    set_PWM(2,1000);
    set_PWM(3,1000);
    set_PWM(0,1000);
  }

  prev_p = local_p;

}

void init_pwm()
{

    pwm=wiringPiI2CSetup (0x40);
    if(pwm==-1)
    {
      printf("-----cant connect to I2C device %d --------\n",pwm);
     
    }
    else
    {
  
      float freq =400.0*.95;
      float prescaleval = 25000000;
      prescaleval /= 4096;
      prescaleval /= freq;
      prescaleval -= 1;
      uint8_t prescale = floor(prescaleval+0.5);
      int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
      int sleep	= settings | 0x10;
      int wake 	= settings & 0xef;
      int restart = wake | 0x80;
      wiringPiI2CWriteReg8(pwm, 0x00, sleep);
      wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
      wiringPiI2CWriteReg8(pwm, 0x00, wake);
      delay(10);
      wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
    }
}

void init_motor(uint8_t channel)
{
	int on_value=0;

	int time_on_us=900;
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1200;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1000;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

}

void set_PWM( uint8_t channel, float time_on_us)
{
  if(run_program==1)
  {
    if(time_on_us>PWM_MAX)
    {
      time_on_us=PWM_MAX;
    }
    else if(time_on_us<1000)
    {
      time_on_us=1000;
    }
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
  else
  {  
    time_on_us=1000;   
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
}

void safety_check(){
  static int hb_count;
  Keyboard keyboard=*shared_memory;

  int current_hb = keyboard.sequence_num;
  // printf("Previous heartbeat: %d\n", prev_hb);
  // printf("Current heartbeat: %d\n", current_hb);

  // printf("\nkeypress is = %c",keyboard.key_press,keyboard.key_press,keyboard.key_press);
  // printf("\nhearbeat %d" ,keyboard.heartbeat);
  // printf("\nhearbeat track %d" ,hb_track);

  // A button Kill
  if(keyboard.keypress == 32){
    // printf("\n A: P down"); 
    // P_button --;
    run_program=0;
  }
  // B button Pause
  if(keyboard.keypress == 33){ 
    // printf("\n B: D up"); 
    // D_button ++;

    pause = true;
  }
  // X button Unpause
  if(keyboard.keypress == 34){ 
    // printf("\n X: D down"); 
    // D_button --;
    pitch_I_term = 0;
    roll_I_term = 0;
    pause = false;
  }
  // Y button Calibrate
  if(keyboard.keypress == 35){ 
    // printf("\n Y: P up"); 
    // P_button ++;
    calibrate_imu();
  }
  
  


  if (roll_position>45 || roll_position<-45){
    printf("\nRoll is TOO XTREME!:ending program\n\r");
    run_program=0;
  }
  if (pitch_position>45 || pitch_position<-45){
    printf("\nPitch is TOO XTREME!: ending program\n\r");
    run_program=0;
  }
  if (imu_data[0]>500 ){//back300
    printf("\n Gyro X detected XTREME rotation!: ending program\n\r");
    run_program=0;
  }

  if ( imu_data[1]>500 ){//back300
    printf("\n Gyro Y detected XTREME rotation!: ending program\n\r");
    run_program=0;
  }

  if (imu_data[2]>500){
    printf("\n Gyro Z detected XTREME rotation!: ending program\n\r");
    run_program=0;
  }

  // add this back later
  if (keyboard.keypress == 32){
    printf("\nYou pressed spacebar: ending program\n\r");
    run_program=0;
  }

  if(current_hb == prev_hb){
    hb_count = hb_count + 1;
    // printf("\nhb_count is %d \n",hb_count);
    if(hb_count > HB_LIM){
      printf("\nHeartbeat not detected!: ending program\n\r");
      run_program=0;
    }

  }else{
    hb_count = 0;
    // printf("\nheartbeat is fine\n");
  }
  prev_hb = current_hb;

}

void setup_keyboard()
{

  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = 0x6400; 
  int smhkey=33222;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  shared_memory = (Keyboard*) shmat (segment_id, 0, 0); 
  printf ("shared memory attached at address %p\n", shared_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  //sprintf (shared_memory, "test!!!!."); 

}

//when cntrl+c pressed, kill motors
void trap(int signal)
{
 
    printf("Killing motors!: ending program in trap\n\r");
    set_PWM(1,1000);
    set_PWM(2,1000);
    set_PWM(3,1000);
    set_PWM(0,1000);



    run_program=0;
}

void calibrate_imu()
{
  x_gyro_calibration= 0;
  y_gyro_calibration= 0;
  z_gyro_calibration= 0;
  // set current position to vive current offsets
  desired_p.x = local_p.x;
  desired_p.y = local_p.y;
  float roll_tot = 0, pitch_tot = 0;
  float z_accel = 0;
  float x_gyro = 0, y_gyro = 0, z_gyro = 0;
  for (int i = 0; i < 1000; i++){
    read_imu();
    roll_tot = roll_tot + roll_angle;
    pitch_tot = pitch_tot + pitch_angle;
    z_accel = z_accel + imu_data[5];
    x_gyro = x_gyro + imu_data[0];
    y_gyro = y_gyro + imu_data[1];
    z_gyro = z_gyro + imu_data[2];
  }

  x_gyro_calibration= x_gyro/-1000.0;
  y_gyro_calibration= y_gyro/-1000.0;
  z_gyro_calibration= z_gyro/-1000.0;
  roll_calibration= roll_tot/-1000.0;
  pitch_calibration= pitch_tot/-1000.0;
  accel_z_calibration= z_accel/-1000.0;
  
  printf("calibration complete\nx_gyro: %f\ny_gyro: %f\nz_gyro: %f\nroll: %f\npitch: %f\nz_accel: %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);
}

void read_imu()
{
  int address=59;//todo: set address value for accel x value 
  float ax=0;
  float az=0;
  float ay=0; 
  int vh,vl;
  
  //read in data
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  //convert 2 complement
  int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[3]=((float)vw/32767.0)*2.0;//  todo: convert vw from raw values to "g's"
  
  
  address=61;//todo: set address value for accel y value
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[4]=((float)vw/32767.0)*2.0;//Todo: convert vw from raw values to "g's"
  
  
  address=63;//todo: set addres value for accel z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[5]= accel_z_calibration + ((float)vw/32767.0)*2.0;//todo: convert vw from raw values to g's
  
  
  address=67;//todo: set addres value for gyro x value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[0]=x_gyro_calibration+((float)vw/32767.0)*500;////todo: convert vw from raw values to degrees/second
  
  address=69;//todo: set addres value for gyro y value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
 imu_data[1]=y_gyro_calibration+((float)vw/32767.0)*500;////todo: convert vw from raw values to degrees/second
  
  address=71;////todo: set addres value for gyro z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[2]=z_gyro_calibration+((float)vw/32767.0)*500;////todo: convert vw from raw values to degrees/second
  
  pitch_angle = pitch_calibration + atan2(-imu_data[4],-imu_data[5]+accel_z_calibration)*(180/3.14159); //pitch_position
  roll_angle = roll_calibration + atan2(imu_data[3],-imu_data[5]+accel_z_calibration)*(180/3.14159); //roll_position

  // printf("Gyro  X:%5.2f   Y:%5.2f   Z:%5.2f     Accel  X:%5.2f   Y:%5.2f   Z:%5.2f     Roll:%5.2f     Pitch:%5.2f  \n", imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4], imu_data[5], roll_angle, pitch_angle);
}

void update_filter()
{
  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev;           
  
  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;
  
  //comp. filter for roll_position, pitch_position here: 
  

  double A=0.004; //0.001


  roll_gyro_delta = imu_data[1]*imu_diff;//deg delta
  pitch_gyro_delta = imu_data[0]*imu_diff;
  yaw_gyro_delta = imu_data[2];


  yaw_gyro = yaw_gyro + yaw_gyro_delta;



  roll_gyro = roll_gyro + roll_gyro_delta;
  pitch_gyro = pitch_gyro + pitch_gyro_delta;

  roll_position = roll_angle*A+(1-A)*(roll_gyro_delta+roll_position);
  pitch_position = pitch_angle*A+(1-A)*(pitch_gyro_delta+pitch_position);

  // fprintf(file_roll,"\nPitch: accel, %5.2f , gyro, %5.2f , filtered, %5.2f  , ROLL: accel, %5.2f , gyro, %5.2f , filtered, %5.2f",pitch_angle, pitch_gyro , pitch_position, roll_angle, roll_gyro , roll_position);
  // fprintf(plot,"\n Pitch: accel, %5.2f , filtered, %5.2f , pwm front, %d, pwm back , %d", pitch_angle, pitch_position, motor0_pwm, motor1_pwm);
  // printf("\nPitch: accel, %5.2f , gyro, %5.2f , filtered, %5.2f  , ROLL: accel, %5.2f , gyro, %5.2f , filtered, %5.2f",pitch_angle, pitch_gyro , pitch_position, roll_angle, roll_gyro , roll_position);
  // fprintf(plot,"\n Pitch:, %5.2f , p_desired, %5.2f , roll, %5.2f , des_roll_joy, %5.2f , yaw angle, %5.2f , yaw desired direction, %5.2f , yaw gyro delta, %5.2f , motor 0 , %5.2f, motor 1, %5.2f , motor2 , %5.2f, motor 3, %5.2f ", pitch_position, des_pitch_joy, roll_position, des_roll_joy, yaw_gyro, des_yaw_joy, yaw_gyro_delta,motor0_pwm,motor1_pwm,motor2_pwm,motor3_pwm);
  fprintf(plot,"\n  yaw angle, %5.2f , yaw desired direction, %5.2f , yaw gyro delta, %5.2f , motor 0 , %d, motor 1, %d , motor2 , %d, motor 3, %d ",  yaw_gyro, des_yaw_joy, yaw_gyro_delta,motor0_pwm,motor1_pwm,motor2_pwm,motor3_pwm);

}

int setup_imu()
{
  wiringPiSetup ();
  
  //setup imu on I2C
  imu=wiringPiI2CSetup (0x68) ; //accel/gyro address
  
  if(imu==-1)
  {
    printf("-----cant connect to I2C device %d --------\n",imu);
    return -1;
  }
  else
  {
  
    printf("connected to i2c device %d\n",imu);
    printf("imu who am i is %d \n",wiringPiI2CReadReg8(imu,0x75));
    
    uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
    
    //init imu
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x00);
    printf("                    \n\r");
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00);  
    wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04        
    int c=wiringPiI2CReadReg8(imu,  GYRO_CONFIG);
    wiringPiI2CWriteReg8(imu,  GYRO_CONFIG, c & ~0xE0);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);       
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c | Ascale << 3);      
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);         
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2, c & ~0x0F); //
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2,  c | 0x00);
  }
  return 0;
}


