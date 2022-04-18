// ##### Include
#include <LiquidCrystal_I2C.h> // For LCD display HD44780 2004 LCD, 4x20 characters

// ##### Configure LCD display, address 0x27 (39), 4x20 Zeichen
LiquidCrystal_I2C lcd(0x27,20,4);

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

// ##### Setup
void setup()
{
  // ##### Initialize USB Serial port
  Serial.begin(9600);
  Serial.println("Startup");

  // ##### Initialize serial 1, UART port, RS485
  // UART, Universal Asynchronous Receiver / Transmitter
  // RX default pin 0
  // TX default pin 1
  // Transmit Enable could be any pin
  #define HWSERIAL Serial1
  HWSERIAL.setRX(0); // Pin 0 = RX
  HWSERIAL.setTX(1); // Pin 1 = TX
  HWSERIAL.transmitterEnable(2); // Pin 2 = Transmitter enable
  HWSERIAL.setTimeout(1000); // Read timeout value in ms
  HWSERIAL.begin(115200); // 115200 9600 baud

  // ##### Digital outputs
  pinMode(LED_BUILTIN, OUTPUT); // LED on board
  
  // ##### Initialize LCD display, 4x20 digits
  lcd.init();
  lcd.backlight();
}

// ##### Loop
void loop()
{
  // ##### Variables
  static bool ledon = false;
  bool WriteData = false;
  static int incomingByteUSB = 0;
  static int LCDBytesReceived;

  // ##### USB read serial port
  if (Serial.available() > 0)
  {
    incomingByteUSB = Serial.read();
    WriteData = true; // Start RS485 write data
  }

  // ##### RS485 write data
  if(WriteData == true)
  {
    WriteData = false;
    // ##### Local variables
    #define BYTES_TENNSY_TO_RASPBERRY 24 // Teensy → Raspberry, Byte 0..23, 24 Byte
    unsigned char t[BYTES_TENNSY_TO_RASPBERRY]= ""; // Empty array where to put the data
    unsigned int Crc;
    unsigned char CrcLByte, CrcHByte;
    int i;

    // Write anything to send data, 24 Byte, Teensy -> Raspberry
    t[0] = 97; // Dec 97 = a
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
    
    // Calculate CRC16
    Crc=0xFFFF;
    for (i=0; i<BYTES_TENNSY_TO_RASPBERRY-2; i++)
    {
      Crc = CRC16(Crc, t[i]);
    }
    CrcLByte = (Crc & 0x00FF);
    CrcHByte = (Crc & 0xFF00) / 256;
    t[22] = CrcLByte;
    t[23] = CrcHByte;

    // Write data to RS485
    HWSERIAL.write(t, BYTES_TENNSY_TO_RASPBERRY);

    // Toggle LED on board
    ledon = ! ledon;
    digitalWrite(LED_BUILTIN, ledon);    
  }

  // ##### RS485 read data
  if (HWSERIAL.available() > 0)
  {
    // ##### Local variables
    #define BYTES_RASPBERRY_TO_TENNSY 33 // Raspberry → Teensy, Byte 0..32, 33 Byte
    unsigned char t[BYTES_RASPBERRY_TO_TENNSY]= ""; // Empty array where to put the data
    int BytesReceived = 0;
    unsigned int Crc;
    unsigned char CrcLByte, CrcHByte;
    int i;

    // Read bytes until
    // 1. The amount of bytes has read
    // 3. It times out (1000ms), 
    BytesReceived = HWSERIAL.readBytes(t, BYTES_RASPBERRY_TO_TENNSY);
    HWSERIAL.clear(); // Discard any received data that has not been read

    LCDBytesReceived = BytesReceived;

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
        
        // Toggle LED on board
        ledon = ! ledon;
        digitalWrite(LED_BUILTIN, ledon);
      }
    }
  }

  // ##### LCD Display
  static char str[80] = "";
  
  sprintf(str,"RS485 received: %i", LCDBytesReceived);
  lcd.setCursor(0,0);  
  lcd.print(str);
}
