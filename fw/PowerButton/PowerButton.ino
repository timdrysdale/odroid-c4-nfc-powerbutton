#include <SPI.h>
#include <MFRC522.h>
#define RST_PIN 9 // Configurable, see typical pin layout above
#define SS_PIN 10 // Configurable, see typical pin layout above
#define IRQ_PIN 2 // Configurable, depends on hardware
MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
MFRC522::MIFARE_Key key;
volatile bool bNewInt = false;
byte regVal = 0x7F;

void setup() {
  
  Serial.begin(57600); // Initialize serial communications with the PC

  SPI.begin(); // Init SPI bus

  mfrc522.PCD_Init(); // Init MFRC522 card
  
  
  pinMode(IRQ_PIN, INPUT_PULLUP); // Setup the IRQ pin
  
  // Allow the irq to be propagated to the IRQ pin
  regVal = 0xA0; // Rx IRQ
  mfrc522.PCD_WriteRegister(mfrc522.ComIEnReg, regVal);
  bNewInt = false; // Interrupt flag
  
  // Activate the interrupt
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), readCard, FALLING);
  /*
  do { // Clear a spourious interrupt at start
    ;
  } while (!bNewInt);
  
  bNewInt = false;
  */
  
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
