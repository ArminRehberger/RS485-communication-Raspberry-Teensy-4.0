# RS485-communication-Raspberry-Teensy4.0
RS485 communication between Raspberry Pi 3 B and Teensy4.0

Notes:

Raspberry Pi 3 B, config, bloototh must be disabled, see functional description

Raspberry Pi 3 B, settings serial port with Linux11 (bullseye), see functional description

Teensy, with simultaneous LiquidChristal, performance drops drastically
#

**Example programs for RS485 communication between Raspberry Pi 3 B and Teensy4.0**


**ASCII Example with hardware flow control:**

Communication is started by typing a character in the serial monitor of the Arduino software (Setting with no line end).

End character = LineFeed \n

Raspberry: A_RS485.py

Teensy: A_RS485_Teensy.ino
#

**Decimal Example with hardware flow control:**

Communication is started by typing a character in the serial monitor of the Arduino software (Setting with no line end).

Communicates a fixed amount of Bytes.

Last two Bytes are usede for CRC16 checksum.

Raspberry: B_RS485.py

Teensy: B_RS485_Teensy.ino
#

**Decimal Example with hardware flow control:**

Communication is started automatically.

Communicates a fixed amount of Bytes.

Last two Bytes are usede for CRC16 checksum.

Raspberry: C_RS485.py

Teensy: C_RS485_Teensy.ino
#

**Decimal Example with automatic flow control:**

Communication is started automatically.

Communicates a fixed amount of Bytes.

Last two Bytes are usede for CRC16 checksum.

Raspberry: D_RS485.py

Teensy: D_RS485_Teensy.ino

Example D: Reached baudrate 500000 bits/s, 0.0002% error


