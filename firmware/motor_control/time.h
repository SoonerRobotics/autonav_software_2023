#include "TickTwo.h"


void setFiveMilliSecFlag();
void setTwentyMilliSecFlag();
void setFiftyMilliSecFlag();
void setFiveHundMilliSecFlag();

bool FIVE_MS_FLAG = false;   //TODO : flag to keep track of 5 ms flag
bool TWENTY_MS_FLAG = false;    //TODO : flag to keep track of 20 ms flag
bool FIFTY_MS_FLAG = false;  //TODO: flag to keep track of 50 ms flag
bool FIVE_HUND_MS_FLAG = false;

void setFiveMilliSecFlag() {
  FIVE_MS_FLAG = true;
}

void setTwentyMilliSecFlag() {
  TWENTY_MS_FLAG = true;
}

void setFiftyMilliSecFlag() {
  FIFTY_MS_FLAG = true;
}
void setFiveHundMilliSecFlag() {
  FIVE_HUND_MS_FLAG = true;
}
