#ifndef NRFUSER_H_INCLUDED
#define NRFUSER_H_INCLUDED

#include <Arduino.h>

bool checkThings(uint8_t retryCnt);
void messageBuild(char* message,uint8_t* messageLength);
void importData(char* data,uint8_t dataLength);
void userHardSetup();
void userHardPowerDown();


#endif // NRFUSER_H_INCLUDED
