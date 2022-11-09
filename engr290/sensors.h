//sensor header, includes the sampling of all distance sensors and IMU

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
