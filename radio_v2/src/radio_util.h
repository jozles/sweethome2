
void marker(uint8_t markerPin);
void markerL(uint8_t markerPin);



#ifdef MACHINE_CONCENTRATEUR

uint8_t cRegister(char* message,uint8_t pldL);
uint8_t macSearch(char* mac,int* numPer);
uint8_t extDataStore(uint8_t numPer,uint8_t numT,uint8_t offset,char* data,uint8_t len);
void tableCPrint();
void tableCInit();
int tableCLoad();
int tableCSave();
int get_radio_message(byte* messageIn,uint8_t* pipe,uint8_t* pldLength);

#endif // MACHINE_CONCENTRATEUR



#ifdef MACHINE_DET328

uint8_t sleepDly(int32_t dly,int32_t* slpt);
uint8_t sleepDly(int32_t dly);
void medSleepDly(int32_t dly);
void hardwarePwrUp();


#endif // MACHINE_DET328

