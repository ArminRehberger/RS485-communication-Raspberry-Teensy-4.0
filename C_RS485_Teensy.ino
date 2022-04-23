// ##### Include
#include <LiquidCrystal_I2C.h> // For LCD display HD44780 2004 LCD, 4x20 characters

// ##### Configure LCD display, address 0x27 (39), 4x20 Zeichen
LiquidCrystal_I2C lcd(0x27,20,4);

#define SLAVE_ADDRESS 17 // RS485 slave address, communication to Raspberry PI, address 17

// ################################################################################
// ##### Function CRC16 checksum calculator
unsigned int CRC16(unsigned int crc, unsigned int data)
{
  const unsigned int Poly16=0xA001;
  unsigned int LSB, i;
  crc = ((crc^data) | 0xFF00) & (crc | 0x00FF);
  for (i=0; i<8; i++)
  {
    LSB=(crc & 0x0001);
    crc=crc/2;
    if (LSB)
      crc=crc^Poly16;
  }
  return(crc);
}

// ################################################################################
// ##### Setup
void setup()
{
  // ##### Initialize USB Serial port
  Serial.begin(9600);
  Serial.println("Startup");

  // ##### Initialize serial 4, UART port, RS485
  // UART, Universal Asynchronous Receiver / Transmitter
  // RX4 default pin 16
  // TX4 default pin 17
  // Transmit Enable could be any pin
  #define HWSERIAL Serial4
  HWSERIAL.setRX(16); // Pin 16 = RX4
  HWSERIAL.setTX(17); // Pin 17 = TX4
  HWSERIAL.transmitterEnable(0); // Pin 0 = Transmitter enable
  HWSERIAL.setTimeout(1000); // Read timeout value in ms
  HWSERIAL.begin(115200); // 115200 9600 baud

  // ##### Digital outputs
  pinMode(LED_BUILTIN, OUTPUT); // LED on board
  
  // ##### Initialize LCD display, 4x20 digits
  lcd.init();
  lcd.backlight();
}

// ################################################################################
// ##### Loop
void loop()
{
  // ##### Variables
  static bool ledon = false;
  static bool WriteData = false;
  static int CounterWriteRS485 = 0;
  static int CounterReadGoodRS485 = 0;
  static int LCDBytesReceived;

  // ################################################################################
  // ##### RS485 write data
  /*
  Teensy → Raspberry, Byte 0..24, 25 Byte
  */
  
  if(WriteData == true)
  {
    WriteData = false;
    // ##### Local variables
    #define BYTES_TENNSY_TO_RASPBERRY 25 // Teensy → Raspberry, Byte 0..24, 25 Byte
    unsigned char t[BYTES_TENNSY_TO_RASPBERRY]= ""; // Empty array where to put the data
    unsigned int Crc;
    unsigned char CrcLByte, CrcHByte;
    int i;

    // Write anything to send data, 25 Byte, Teensy -> Raspberry
    t[0] = SLAVE_ADDRESS;
    t[1] = 98;
    t[2] = 99;
    t[3] = 100;
    t[4] = 101;
    t[5] = 102;
    t[6] = 103;
    t[7] = 104;
    t[8] = 105;
    t[9] = 106;
    t[10] = 107;
    t[11] = 108;
    t[12] = 109;
    t[13] = 110;
    t[14] = 111;
    t[15] = 112;
    t[16] = 113;
    t[17] = 114;
    t[18] = 115;
    t[19] = 116;
    t[20] = 117;
    t[21] = 118;
    t[22] = 119;
    
    // Calculate CRC16
    Crc=0xFFFF;
    for (i=0; i<BYTES_TENNSY_TO_RASPBERRY-2; i++)
    {
      Crc = CRC16(Crc, t[i]);
    }
    CrcLByte = (Crc & 0x00FF);
    CrcHByte = (Crc & 0xFF00) / 256;
    t[23] = CrcLByte;
    t[24] = CrcHByte;

    // Write data to RS485
    HWSERIAL.write(t, BYTES_TENNSY_TO_RASPBERRY);
    CounterWriteRS485++;

    // Toggle LED on board
    ledon = ! ledon;
    digitalWrite(LED_BUILTIN, ledon);    
  }

  // ################################################################################
  // ##### RS485 read data
  /*
  Raspberry → Teensy, Byte 0..33, 34 Byte
   */
  if (HWSERIAL.available() > 0)
  {
    // ##### Local variables
    #define BYTES_RASPBERRY_TO_TENNSY 34 // Raspberry → Teensy, Byte 0..33, 34 Byte
    unsigned char t[BYTES_RASPBERRY_TO_TENNSY]= ""; // Empty array where to put the data
    int BytesReceived = 0;
    unsigned int Crc;
    unsigned char CrcLByte, CrcHByte;
    int i;
    static unsigned long DelaytimeMillis = 5; // Delaytime in ms between two data packets 5ms (read -> write)
    static unsigned long previousMillis = 0;
    static unsigned long currentMillis = 0;

    // Read bytes until
    // 1. The amount of bytes has read
    // 3. It times out (1000ms), 
    BytesReceived = HWSERIAL.readBytes(t, BYTES_RASPBERRY_TO_TENNSY);
    HWSERIAL.clear(); // Discard any received data that has not been read

    LCDBytesReceived = BytesReceived;

    // Check slave address
    if(t[0] == SLAVE_ADDRESS)
    {
      // Check amount of received bytes
      if(BytesReceived == BYTES_RASPBERRY_TO_TENNSY)
      {
        // Calculate CRC16
        Crc=0xFFFF;
        for (i=0; i<BYTES_RASPBERRY_TO_TENNSY-2; i++)
        {
          Crc = CRC16(Crc, t[i]);
        }
        CrcLByte = (Crc & 0x00FF);
        CrcHByte = (Crc & 0xFF00) / 256;

        // Check CRC16
        if(t[BYTES_RASPBERRY_TO_TENNSY-2] == CrcLByte && t[BYTES_RASPBERRY_TO_TENNSY-1] == CrcHByte)
        {
          // Print the received data to USB serial port
          char charValue[1];
          for(i = 0; i < BytesReceived; i++)
          {
            sprintf(charValue, "%c", t[i]);
            Serial.print(charValue);
          }
          Serial.println("");
 
          // Delaytime in ms between two data packets 5ms (read -> write)
          currentMillis = millis();
          previousMillis = currentMillis;
          do
          {
            currentMillis = millis();
          } while (currentMillis - previousMillis < DelaytimeMillis);    

          // Start next write data RS485
          WriteData = true; // Start RS485 write data
          CounterReadGoodRS485++;

          // Toggle LED on board
          ledon = ! ledon;
          digitalWrite(LED_BUILTIN, ledon);
        }
      }
    }
  }

  // ################################################################################
  // ##### LCD Display
  static char str[80] = "";
  
  sprintf(str,"RS485 received: %i", LCDBytesReceived);
  lcd.setCursor(0,0);  
  lcd.print(str);

  sprintf(str,"Write: %i", CounterWriteRS485);
  lcd.setCursor(0,1);  
  lcd.print(str);

  sprintf(str,"Read: %i", CounterReadGoodRS485);
  lcd.setCursor(0,2);  
  lcd.print(str);
}
