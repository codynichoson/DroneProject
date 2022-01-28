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
#define PWM_MAX          1700
#define NEUTRAL_PWR      1350
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
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
float roll = 0; // self add
float pitch = 0;// self add
float roll_gyro = 0;// self add
float pitch_gyro = 0;// self add
int prev_hb = 0;
int pwm;

float roll_gyro_delta;
float pitch_gyro_delta;
float pitch_I_term;

int motor0_pwm;
int motor1_pwm;
int motor2_pwm;
int motor3_pwm;

struct Keyboard {
  char key_press;
  int heartbeat;
  int version;
};
Keyboard* shared_memory;
int run_program=1;

// FILE *file_roll = fopen("roll.csv", "w");
FILE *plot = fopen("plot.csv", "w");

int main (int argc, char *argv[])
{
    //in main function before calibrate imu add
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
      // printf("\nwe rolling"); 
    }
    return 0;
}

void motor_pwm(){
  
  // gains
  int P = 15;
  int D = 690;
  float I = 0.042069;

  // motor1_pwm = NEUTRAL_PWR + pitch*P;     // p controller
  // motor2_pwm = NEUTRAL_PWR + pitch*P;

  // motor3_pwm = NEUTRAL_PWR - pitch*P;
  // motor0_pwm = NEUTRAL_PWR - pitch*P;

  // motor1_pwm = NEUTRAL_PWR + pitch_gyro_delta*D; // D controler
  // motor2_pwm = NEUTRAL_PWR + pitch_gyro_delta*D;

  // motor3_pwm = NEUTRAL_PWR - pitch_gyro_delta*D;
  // motor0_pwm = NEUTRAL_PWR - pitch_gyro_delta*D;

  // motor1_pwm = NEUTRAL_PWR + pitch_gyro_delta*D + pitch*P ; // PD controller
  // motor2_pwm = NEUTRAL_PWR + pitch_gyro_delta*D+ pitch*P ;

  // motor3_pwm = NEUTRAL_PWR - pitch_gyro_delta*D - pitch*P ;
  // motor0_pwm = NEUTRAL_PWR - pitch_gyro_delta*D - pitch*P ;
  
  pitch_I_term += pitch*I;

  if(pitch_I_term > 150){
    pitch_I_term = 150;
  }
  if(pitch_I_term < -150){
    pitch_I_term = -150;
  }
  printf("\npitch I term %f : pitch is %f",pitch_I_term,pitch);

  motor1_pwm = NEUTRAL_PWR + pitch_gyro_delta*D + pitch*P + pitch_I_term;
  motor2_pwm = NEUTRAL_PWR + pitch_gyro_delta*D + pitch*P + pitch_I_term;

  motor3_pwm = NEUTRAL_PWR - pitch_gyro_delta*D - pitch*P - pitch_I_term;
  motor0_pwm = NEUTRAL_PWR - pitch_gyro_delta*D - pitch*P - pitch_I_term;

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
  set_PWM(1,motor1_pwm);
  set_PWM(2,motor2_pwm);
  set_PWM(3,motor3_pwm);
  set_PWM(0,motor0_pwm);

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

  int current_hb = keyboard.heartbeat;
  // printf("Previous heartbeat: %d\n", prev_hb);
  // printf("Current heartbeat: %d\n", current_hb);

  // printf("\nkeypress is = %c",keyboard.key_press,keyboard.key_press,keyboard.key_press);
  // printf("\nhearbeat %d" ,keyboard.heartbeat);
  // printf("\nhearbeat track %d" ,hb_track);

  if (roll>45 || roll<-45){
    printf("\nRoll is TOO XTREME!:ending program\n\r");
    run_program=0;
  }
  if (pitch>45 || pitch<-45){
    printf("\nPitch is TOO XTREME!: ending program\n\r");
    run_program=0;
  }
  if (imu_data[0]>300 ){
    printf("\n Gyro X detected XTREME rotation!: ending program\n\r");
    run_program=0;
  }

  if ( imu_data[1]>300 ){
    printf("\n Gyro Y detected XTREME rotation!: ending program\n\r");
    run_program=0;
  }

  if (imu_data[2]>300){
    printf("\n Gyro Z detected XTREME rotation!: ending program\n\r");
    run_program=0;
  }

  if (keyboard.key_press == 32){
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
  
  pitch_angle = pitch_calibration + atan2(-imu_data[4],-imu_data[5]+accel_z_calibration)*(180/3.14159); //pitch
  roll_angle = roll_calibration + atan2(imu_data[3],-imu_data[5]+accel_z_calibration)*(180/3.14159); //roll

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
  
  //comp. filter for roll, pitch here: 
  

  double A=0.001;
  roll_gyro_delta = imu_data[1]*imu_diff;
  pitch_gyro_delta = imu_data[0]*imu_diff;

  roll_gyro = roll_gyro + roll_gyro_delta;
  pitch_gyro = pitch_gyro + pitch_gyro_delta;

  roll = roll_angle*A+(1-A)*(roll_gyro_delta+roll);
  pitch = pitch_angle*A+(1-A)*(pitch_gyro_delta+pitch);

  // fprintf(file_roll,"\nPitch: accel, %5.2f , gyro, %5.2f , filtered, %5.2f  , ROLL: accel, %5.2f , gyro, %5.2f , filtered, %5.2f",pitch_angle, pitch_gyro , pitch, roll_angle, roll_gyro , roll);
  fprintf(plot,"\n Pitch: accel, %5.2f , filtered, %5.2f , pwm front, %d, pwm back , %d", pitch_angle, pitch, motor0_pwm, motor1_pwm);
  // printf("\nPitch: accel, %5.2f , gyro, %5.2f , filtered, %5.2f  , ROLL: accel, %5.2f , gyro, %5.2f , filtered, %5.2f",pitch_angle, pitch_gyro , pitch, roll_angle, roll_gyro , roll);

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


