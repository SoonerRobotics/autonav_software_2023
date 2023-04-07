#include "TickTwo.h"


void setFiveMilliSecFlag();
void setTenMilliSecFlag();
void setFiftyMilliSecFlag();
void setFiveHundMilliSecFlag();

bool FIVE_MS_FLAG = false;   //TODO : flag to keep track of 5 ms flag
bool TEN_MS_FLAG = false;    //TODO : flag to keep track of 10 ms flag
bool FIFTY_MS_FLAG = false;  //TODO: flag to keep track of 50 ms flag
bool FIVE_HUND_MS_FLAG = false;

void setFiveMilliSecFlag() {
  FIVE_MS_FLAG = true;
}

void setTenMilliSecFlag() {
  TEN_MS_FLAG = true;
}

void setFiftyMilliSecFlag() {
  FIFTY_MS_FLAG = true;
}
void setFiveHundMilliSecFlag() {
  FIVE_HUND_MS_FLAG = true;
}
