// IMPORT LIBRARIES
#include <SPI.h>
#include <MFRC522.h>

// NFC 
#define RST_PIN 9 // Configurable, see typical pin layout above
#define SS_PIN 10 // Configurable, see typical pin layout above
#define IRQ_PIN 2 // Configurable, depends on hardware
MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
MFRC522::MIFARE_Key key;
volatile bool bNewInt = false;
byte regVal = 0x7F;

// MFRC522 interrupt serving routine
  void readCard() {
  bNewInt = true;
}
// Helper routine to dump a byte array as hex values to Serial
void dump_byte_array(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

// The function sending to the MFRC522 the needed commands to activate the reception
void activateRec(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg,
  mfrc522.PICC_CMD_REQA);
  mfrc522.PCD_WriteRegister(mfrc522.CommandReg,
  mfrc522.PCD_Transceive);
  mfrc522.PCD_WriteRegister(mfrc522.BitFramingReg, 0x87);
}

// The function to clear the pending interrupt bits after ISR
void clearInt(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.ComIrqReg, 0x7F);
}


void setup() {

 
  //Serial communication for sending data -> RPi -> Server
  Serial.begin(57600);
  while(!Serial);

  // NFC setup
  SPI.begin(); // Init SPI bus
  
  mfrc522.PCD_Init(); // Init MFRC522 card
  
  pinMode(IRQ_PIN, INPUT_PULLUP); // Setup the IRQ pin
  
  // Allow the irq to be propagated to the IRQ pin
  regVal = 0xA0; // Rx IRQ
  mfrc522.PCD_WriteRegister(mfrc522.ComIEnReg, regVal);
  bNewInt = false; // Interrupt flag
  
  // Activate the interrupt
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), readCard, FALLING);

}

void loop() {

  // Read and printout the MFRC522 version (valid values 0x91 & 0x92)
  Serial.print(F("Ver: 0x"));
  
  byte readReg = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  Serial.println(readReg, HEX);
  
  Serial.println("Hi!");
  delay(500);
  
  if (bNewInt) { // New read interrupt
    Serial.print(F("Interrupt. "));
    mfrc522.PICC_ReadCardSerial(); // Read the tag data
  
    // Show some details of the PICC
    Serial.print(F("Card UID:"));
    dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
    Serial.println();
    clearInt(mfrc522);
    mfrc522.PICC_HaltA();
    bNewInt = false;
  }
  
  activateRec(mfrc522);
  delay(100);
  
}


/*
// Firmware for power button
// Single controller firmware with state machine for reading and writing.

//Author: Timothy D. Drysdale
// timothy.d.drysdale@gmail.com
// 01/10/2022

// IMPORT LIBRARIES
#include "ArduinoJson-v6.9.1.h"
#include <FlashStorage.h>
#include <SPI.h>
#include <MFRC522.h>

// NFC 
#define RST_PIN 9 // Configurable, see typical pin layout above
#define SS_PIN 10 // Configurable, see typical pin layout above
#define IRQ_PIN 2 // Configurable, depends on hardware
MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
MFRC522::MIFARE_Key key;
volatile bool bNewInt = false;
byte regVal = 0x7F;

// JSON serialization
#define COMMAND_SIZE 192  //https://arduinojson.org/v6/assistant
StaticJsonDocument<COMMAND_SIZE> doc;
char command[COMMAND_SIZE];

#define FLASH_WRITES_MAX 3
#define SECRET_LEN_MAX 99
#define SCALE_FACTOR_LEN 7
const size_t SCALE_FACTOR_CAPACITY = JSON_ARRAY_SIZE(SCALE_FACTOR_LEN);
StaticJsonDocument<SCALE_FACTOR_CAPACITY> scale_factors;

// calibration value indices in cal array
#define LOAD_CELL 0
#define MEMBER_1 1
#define MEMBER_2 2
#define MEMBER_3 3
#define MEMBER_4 4
#define MEMBER_5 5
#define MEMBER_6 6

// Create a structure that contains is big enough to contain a name
// and a surname. The "valid" variable is set to "true" once
// the structure is filled with actual data for the first time.
typedef struct {
  boolean secure;          // Set this true when secret is first written 
  boolean valid;           // Set this true when calibration data is first written
  char secret[SECRET_LEN_MAX];  // Secret string for authorising calibration updates (typically a uuid of 36 chars in form 8-4-4-4-12)
  int  writes;             // Count number of remaining writes we'll permit
  // You can change the values below here to suit your experiment
  float scale_factor[SCALE_FACTOR_LEN];   // Scale factors for some experiment or other
} Calibration;

// Create a global "Calibration" variable and call it cal
Calibration cal;

// Reserve a portion of flash memory to store a "Calibration" and
// call it "cal_store".
FlashStorage(cal_store, Calibration);

// Timer setup for reporting
bool do_report;
float report_interval = 3000; //ms, for timer interrupt

// Timer (specific to nano IOT 33)
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
float timer_interrupt_freq = 1000.0/report_interval;


// Defines the valid states for the state machine

typedef enum
{
  STATE_STARTUP = 0,        // anything to do with starting up the experiment
  STATE_SETSECRET = 1,     // await authorisation setup before accepting calibration data
  STATE_SETCAL = 2,       // await a calibration data before beginning normal operation of the experiment
  STATE_STANDBY = 3,        // Standby for commands relating to normal operation - represents all the states that would normally be associated with normal operation
  STATE_LOADCAL = 4,      // update calibration
} StateType;

//state Machine function prototypes
//these are the functions that run whilst in each respective state.
void Sm_State_StartUp(void);
void Sm_State_SetSecret(void);
void Sm_State_SetCal(void);
void Sm_State_Standby(void);
void Sm_State_LoadCal(void);

// cal function prototypes (here because readSerialJSON has to come after StateType definition)
bool cal_is_secure();
bool cal_is_valid();
void startTimer(int);
void report_cal();
void cal_set_secret(const char *);
void cal_set_values(StaticJsonDocument<COMMAND_SIZE>);
StateType readSerialJSON(StateType); //has to come after type definition


// Type definition used to define the state
 
typedef struct
{
  StateType State; // Defines the command 
  void (*func)(void); // Defines the function to run 
} StateMachineType;


// A table that defines the valid states of the state machine and
// the function that should be executed for each state

StateMachineType StateMachine[] =
{
  {STATE_STARTUP,     Sm_State_StartUp},
  {STATE_SETSECRET,   Sm_State_SetSecret}, 
  {STATE_SETCAL,      Sm_State_SetCal},
  {STATE_STANDBY,     Sm_State_Standby},
  {STATE_LOADCAL,     Sm_State_LoadCal}, 
};
 
int NUM_STATES = 5;

// Stores the current state of the state machine


StateType SmState = STATE_STARTUP;    // Start off in StartUp state 

//DEFINE STATE MACHINE FUNCTIONS================================================================


//TRANSITION: STATE_STARTUP -> STATE_SECURECAL (if calibration is secure and valid)
void Sm_State_StartUp(void){

    // do any other start up tasks required before checking/loading calibration here
    
    SmState = STATE_SETSECRET;

}

//TRANSITION: STATE_SETSECRET -> STATE_SETCAL (if cal is secure)
void Sm_State_SetSecret(void){

  if (cal_is_secure()) {
    SmState = STATE_SETCAL; //we let startup state dictate progress through essential tasks
    }

  SmState = STATE_SETSECRET; 
  
}


//TRANSITION: STATE_SETCAL -> STATE_LOADCAL (if cal is set)
void Sm_State_SetCal(void){

  if (!cal_is_valid()) {
    SmState = STATE_LOADCAL;
    }

  SmState = STATE_SETCAL; 
  
}
//TRANSITION: STATE_LOADCAL -> STATE_STANDBY (if cal is set)
void Sm_State_LoadCal(void){

   
  
}

//TRANSITION: STATE_STANDBY -> STATE_STANDBY
void Sm_State_Standby(void){

  SmState = STATE_STANDBY;
  
}


//STATE MACHINE RUN FUNCTION
void Sm_Run(void)
{
  if (SmState < NUM_STATES)
  {
    SmState = readSerialJSON(SmState);      
    (*StateMachine[SmState].func)();        //reads the current state and then runs the associated function
    
  }
  else{
    Serial.println("Exception in State Machine");
  }
  
}



void setup() {

  startTimer(timer_interrupt_freq);   //setup and start the timer interrupt functions for PID calculations

  //Serial communication for sending data -> RPi -> Server
  Serial.begin(57600);
  while(!Serial);

  // NFC setup
  SPI.begin(); // Init SPI bus
  
  mfrc522.PCD_Init(); // Init MFRC522 card
  
  pinMode(IRQ_PIN, INPUT_PULLUP); // Setup the IRQ pin
  
  // Allow the irq to be propagated to the IRQ pin
  regVal = 0xA0; // Rx IRQ
  mfrc522.PCD_WriteRegister(mfrc522.ComIEnReg, regVal);
  bNewInt = false; // Interrupt flag
  
  // Activate the interrupt
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), readCard, FALLING);

}

void loop() {
    Sm_Run();

    // reporting
    if (do_report) {
        report();
        do_report = false;
      }
}



StateType readSerialJSON(StateType SmState){
  if(Serial.available() > 0)
  {
    
    Serial.readBytesUntil(10, command, COMMAND_SIZE);
    deserializeJson(doc, command);

    const char* get = doc["get"];
    
    if(strcmp(get, "cal")==0) {
      report_cal();
      return SmState; // force {"get":cal"} to be a stand-alone command that can't be combined with set
    }
    
    const char* set = doc["set"];

    if(strcmp(set, "secret")==0)
    {
        const char* secret = doc["to"]; 
        cal_set_secret(secret); // this function can be safely called many times because it ignores all but the first non-empty secret 
    } 
    else if(strcmp(set, "cal")==0)
    {
     cal_set_values(doc);
    } 
    
  } // serial available
  
  return SmState;     //return whatever state it changed to or maintain the state.
  
 } 

void report(){
  Serial.println("{\"some\":\"data\"}");
  }
  
void report_cal(){
  Serial.print("{\"report\":\"cal\",\"secure\":");
  Serial.print(cal.secure);
  Serial.print(",\"valid\":");
  Serial.print(cal.valid);
  Serial.print(",\"writes_left\":");
  Serial.print(cal.writes);  
  Serial.print(",\"sf_load\":"); 
  Serial.print(cal.scale_factor[LOAD_CELL]);
  Serial.print(",\"sf_m1\":"); 
  Serial.print(cal.scale_factor[MEMBER_1]); 
  Serial.print(",\"sf_m2\":"); 
  Serial.print(cal.scale_factor[MEMBER_2]);  
  Serial.print(",\"sf_m3\":"); 
  Serial.print(cal.scale_factor[MEMBER_3]); 
  Serial.print(",\"sf_m4\":"); 
  Serial.print(cal.scale_factor[MEMBER_4]);      
  Serial.print(",\"sf_m5\":"); 
  Serial.print(cal.scale_factor[MEMBER_5]); 
  Serial.print(",\"sf_m6\":"); 
  Serial.print(cal.scale_factor[MEMBER_6]);    
  Serial.print(cal.valid); 
  Serial.println("}");
 
}

bool cal_is_secure(){

    cal = cal_store.read();
  
    return cal.secure;
    
  }

bool cal_is_valid(){

    cal = cal_store.read();
 
    return (cal.secure && cal.valid);
    
  }
  
// set the secret for authorising changes to calibration data
void cal_set_secret(const char *secret){

    if (secret[0] == '\0') { 
      return; //empty string
    }
  
    cal = cal_store.read();
    
    if (cal.secure){ // only set secret once
       Serial.println("{\"error\":\"secret already set\"}");
    } else {
        strncpy(cal.secret, secret, (sizeof cal.secret) - 1);
        cal.secure = true;
        cal.writes = FLASH_WRITES_MAX;
        cal_store.write(cal);
        Serial.println("{\"log\":\"secret\",\"is\":\"set\"}"); 
      }
  }

// set the calibration values
void cal_set_values(StaticJsonDocument<COMMAND_SIZE> doc){
  
      cal = cal_store.read();

      if (!cal.secure) {
        Serial.println("{\"error\":\"cal secret not set\"}");
        return; // don't set values before setting authorisation (prevent rogue writes)
      }

      if (cal.writes <= 0) {
        Serial.println("{\"error\":\"no more cal writes permitted - reflash firmware to reset counter\"}");
        return; // prevent writes if remaining write count has reached zero
      }
      
      const char* secret =  doc["auth"];

      
      if (!(strcmp(cal.secret, secret)==0)) {
          Serial.println("{\"error\":\"wrong secret\"}");
          return; // don't set values if auth code does not match secret
        } 

      JsonArray values = doc["to"];

      if (values.size() != SCALE_FACTOR_LEN) { 
        Serial.print("{\"error\":\"wrong number of values in cal array\",");
        Serial.print("\"want\":");
        Serial.print(SCALE_FACTOR_LEN);
        Serial.print(",\"have\":");
        Serial.print(values.size());
        Serial.println("}");
        return; // don't set cal values if wrong number 
        } //size ok

        cal.writes -= 1;
        cal.valid = true;
                        
        Serial.print("{\"log\":\"cal\",\"is\":\"ok\",\"values\":[");
        for (int i=0; i<SCALE_FACTOR_LEN; i++) {
            cal.scale_factor[i]= values[i];
            Serial.print(cal.scale_factor[i]);
            if (i<(SCALE_FACTOR_LEN-1)){
               Serial.print(",");
            } 
            else 
            {
               Serial.print("],\"writes_remaining\":");
               Serial.print(cal.writes);
               Serial.println("}");
            }
          } // for
        
        cal_store.write(cal);
}
        

bool load_cal(){
  
    cal = cal_store.read();
    
  }
 
//This function is run on a timer interrupt defined by PIDInterval/timer_interrupt_freq.
void TimerInterrupt(void) {
   do_report = true;
}


//===================================================================================
// NFC functions
//===================================================================================

// MFRC522 interrupt serving routine
  void readCard() {
  bNewInt = true;
}
// Helper routine to dump a byte array as hex values to Serial
void dump_byte_array(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

// The function sending to the MFRC522 the needed commands to activate the reception
void activateRec(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg,
  mfrc522.PICC_CMD_REQA);
  mfrc522.PCD_WriteRegister(mfrc522.CommandReg,
  mfrc522.PCD_Transceive);
  mfrc522.PCD_WriteRegister(mfrc522.BitFramingReg, 0x87);
}

// The function to clear the pending interrupt bits after ISR
void clearInt(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.ComIrqReg, 0x7F);
}


//======================TIMER INTERRUPT FUNCTIONS====================================
//      FROM https://github.com/nebs/arduino-zero-timer-demo/
//===================================================================================

void setTimerFrequencyHz(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

//This is a slightly modified version of the timer setup found at:
//https://github.com/maxbader/arduino_tools

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID (GCM_TCC2_TC3)) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 );

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  setTimerFrequencyHz(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we run the TimerInterrupt() function.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    TimerInterrupt();           //THE FUNCTION TO RUN ON THE TIMER
  }
}

*/
