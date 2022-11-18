//sensor header, includes the sampling of all distance sensors and IMU
#define IMU_ADDR 0x68 
#define CONFIG_ADDR 0x1A
#define GYRO_CONFIG_ADDR 0x1B
#define ACCEL_CONFIG_ADDR 0x1C
#define GYRO_OUT_START 0x43
#define ACCEL_OUT_START 0x3B
#define GYRO_250 0x00
#define GYRO_500 0x08
#define GYRO_1000 0x10
#define GYRO_2000 0x18
#define GYRO_XST 0x80
#define GYRO_YST 0x40
#define GYRO_ZST 0x20
#define ACCEL_2G 0x00
#define ACCEL_4G 0x08
#define ACCEL_8G 0x10
#define ACCEL_16G 0x18
#define ACCEL_XST 0x80
#define ACCEL_YST 0x40
#define ACCEL_ZST 0x20
//init function for IMU
void initIMU();

//returns the distance from the left sensor as uint32, actually the count of the timer representing the pulse length
uint32_t getLeftDistance();

//returns the distance from the right sensor as uint32, actually the count of the timer representing the pulse length
uint32_t getRightDistance();

//returns the distance from the front sensor as uint32, unsure about data type as it's new type of sensor
uint32_t getFrontDistance();

//return the angular velocity in the yaw axis as int16_t
int16_t getYawRate();

//initiailze ADC needed for reading IR sensors
void initIRSensors();
