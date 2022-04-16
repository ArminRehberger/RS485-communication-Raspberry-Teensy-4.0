// ##### Include
#include <LiquidCrystal_I2C.h> // For LCD display HD44780 2004 LCD, 4x20 characters

// ##### Configure LCD display, address 0x27 (39), 4x20 Zeichen
LiquidCrystal_I2C lcd(0x27,20,4);

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
  static int BytesRecieved = 0;
  bool WriteData = false;
  static int incomingByteUSB = 0;
  int i;

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
    // Teensy → Raspberry, Byte 0..22, 23 Byte + \r + \n = 25 Bytes
    // \r = carriage return (Dec 13), \n = linefeed (Dec 24) is added automatically
    char t[25]= "00000000000000000000000"; // Empty array where to put the data

    // Write anything to send data, 23 Byte, Teensy -> Raspberry
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
    t[22] = 119;

    HWSERIAL.println(t); // Send string with carriage return and linefeed character \r\n

    // Toggle LED on board
    ledon = ! ledon;
    digitalWrite(LED_BUILTIN, ledon);    
  }

  // ##### RS485 read data
  if (HWSERIAL.available() > 0)
  {
    // Buffer for the received data
    const int BUFFER_SIZE = 100;
    char buf[BUFFER_SIZE];

    // Read bytes until
    // 1. The terminator character is detected (LineFeed \n). The terminator itself is not returned in the buffer.
    // 2. BUFFER_SIZE (100) has reached
    // 3. It times out (1000ms), 
    BytesRecieved = HWSERIAL.readBytesUntil('\n', buf, BUFFER_SIZE);
    HWSERIAL.clear(); // Discard any received data that has not been read
    
    // Print the received data to USB serial port
    for(i = 0; i < BytesRecieved; i++)
      Serial.print(buf[i]);
    Serial.println("");
    
    // Toggle LED on board
    ledon = ! ledon;
    digitalWrite(LED_BUILTIN, ledon);   
  }

  // ##### LCD Display
  static char str[80] = "";
  
  sprintf(str,"RS485 received: %i", BytesRecieved);
  lcd.setCursor(0,0);  
  lcd.print(str);
}
