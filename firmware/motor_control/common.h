

bool FIVE_MS_FLAG = false;   //TODO : flag to keep track of 5 ms flag
bool TEN_MS_FLAG = false;    //TODO : flag to keep track of 10 ms flag
bool FIFTY_MS_FLAG = false;  //TODO: flag to keep track of 50 ms flag

uint32_t ODOM_OUT_ID = 14; 
uint32_t ODOM_OUT_LEN = 6;

//in meters
float DIAMETER_FROM_CENTER_WHEEL = 0.1016;
float DISTANCE_BETWEEN_WHEELS = 0.4826;

uint32_t SPEED_SCALE_FACTOR = 1000;
uint32_t ODOM_SCALE_FACTOR = 10000;


void setFiveMilliSecFlag();
void setTenMilliSecFlag();
void setFiftyMilliSecFlag();

typedef struct{
    unsigned int eStop : 1;
    unsigned int mStop : 1;
    unsigned int mStart : 1;
}robotStatus_t;

typedef  struct{
      short xn;
      short yn;
      short on;
}distance;

typedef struct{
  short setpointLeft;
  short setpointRight;


}motorCommand;

void setFiveMilliSecFlag() {
  FIVE_MS_FLAG = true;
}

void setTenMilliSecFlag() {
  TEN_MS_FLAG = true;
}

void setFiftyMilliSecFlag() {
  FIFTY_MS_FLAG = true;
}
