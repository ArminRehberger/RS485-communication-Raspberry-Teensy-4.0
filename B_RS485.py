""" ################################################ """
""" Import python modules """
import time
import serial
import RPi.GPIO as GPIO

""" ################################################ """
""" GPIO """
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD) # GPIO = Pin numbers
# RS485 Transmitter enable. Pin number 7, GPIO4. DE & RE of RS-485 module 
# Output low=receive, high=send
GPIO.setup(7, GPIO.OUT, initial=GPIO.LOW)

""" ################################################ """
""" Initialize serial 0, RS485 """
# 8N1, 8 Data Bits, No Parity, 1 Stopbit
# 115200 Baud, 
# Read timeout 1.0s
HWSERIAL = serial.Serial(
    port='/dev/serial0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1.0 # Read timeout value in seconds. Value = float
)

""" ################################################ """
""" Function CRC16 checksum calculator """
def CRC16(crc, data):
    Poly16 = 0xA001
    LSB = 0
    i = 0
    
    crc = ((crc ^ data) | 0xFF00) & (crc | 0x00FF)
    for i in range(0, 8):
        LSB = crc & 0x0001
        crc = crc >> 1 # Instead of /2, that value remains as an integer
        if(LSB):
            crc = crc ^ Poly16

    return crc

""" ################################################ """
""" Global variables """
NumberBytesWritten = 0
BytesReceived = 0
CounterRS485ReceivedGood = 0
CounterRS485SendGood = 0
WriteData = False

# Bytearray 0..23, 24 Byte receive from teensy
BYTES_TEENSY_TO_RASPBERRY = 24
ReadDataMotor = bytearray(BYTES_TEENSY_TO_RASPBERRY)

# Bytearray 0..32, 33 Byte send to teensy
BYTES_RASPBERRY_TO_TEENSY = 33
WriteDataMotor = bytearray(BYTES_RASPBERRY_TO_TEENSY)

# Write anything to send data, 33 Byte, Raspberry -> Teensy
WriteByte0 = 65 # Dec 65 = A
WriteByte1 = 66
WriteByte2 = 67
WriteByte3 = 68
WriteByte4 = 69
WriteByte5 = 70
WriteByte6 = 71
WriteByte7 = 72
WriteByte8 = 73
WriteByte9 = 74
WriteByte10 = 75
WriteByte11 = 76
WriteByte12 = 77
WriteByte13 = 78
WriteByte14 = 79
WriteByte15 = 80
WriteByte16 = 81
WriteByte17 = 82
WriteByte18 = 83
WriteByte19 = 84
WriteByte20 = 85
WriteByte21 = 86
WriteByte22 = 87
WriteByte23 = 88
WriteByte24 = 89
WriteByte25 = 90
WriteByte26 = 91
WriteByte27 = 92
WriteByte28 = 93
WriteByte29 = 94
WriteByte30 = 95
WriteByte31 = 96
WriteByte32 = 97

""" ######################################################################################## """
""" Start loop forever """
while True:

    """ ######################################################################################## """
    """ RS485 read data """
    # Read a bytearray
    # Teensy -> Raspberry, Byte 0..BYTES_TEENSY_TO_RASPBERRY
    # Read timeout must be set in Serial, if amount of bytes are not received
    line = HWSERIAL.read(BYTES_TEENSY_TO_RASPBERRY)
    HWSERIAL.reset_input_buffer() # Clear input buffer
    if line:
        BytesReceived = len(line)
        
        # Check amount of received bytes
        if BytesReceived == BYTES_TEENSY_TO_RASPBERRY:
            WriteData = True
            
            # Calculate CRC16
            Crc = 0xFFFF
            CrcLByte = 0
            CrcHByte = 0

            for i in range(0, BYTES_TEENSY_TO_RASPBERRY-2):
                Crc = CRC16(Crc, line[i])
            CrcLByte = Crc & 0x00FF
            CrcHByte = (Crc & 0xFF00) >> 8 # Instead of /256, that value remains as an integer

            # Check CRC16
            if line[BYTES_TEENSY_TO_RASPBERRY-2] == CrcLByte\
            and line[BYTES_TEENSY_TO_RASPBERRY-1] == CrcHByte:
                CounterRS485ReceivedGood +=1
                print(line)
                # Copy Bytes
                for i in range(0, BYTES_TEENSY_TO_RASPBERRY):
                    ReadDataMotor[i] = line[i]

    """ ######################################################################################## """
    """ RS485 write data """
    # Write a bytearray
    # Raspberry -> Teensy, Byte 0..BYTES_RASPBERRY_TO_TEENSY
    if WriteData == True:
        WriteData = False

        # Write the send data to a bytearray, 0..32, 33 Byte
        WriteDataMotor = [WriteByte0, WriteByte1, WriteByte2, WriteByte3, WriteByte4,
                          WriteByte5, WriteByte6, WriteByte7, WriteByte8, WriteByte9,
                          WriteByte10, WriteByte11, WriteByte12, WriteByte13, WriteByte14,
                          WriteByte15, WriteByte16, WriteByte17, WriteByte18, WriteByte19,
                          WriteByte20, WriteByte21, WriteByte22, WriteByte23, WriteByte24,
                          WriteByte25, WriteByte26, WriteByte27, WriteByte28, WriteByte29,
                          WriteByte30, WriteByte31, WriteByte32]

        # Calculate CRC16
        Crc = 0xFFFF
        CrcLByte = 0
        CrcHByte = 0
       
        for i in range(0, BYTES_RASPBERRY_TO_TEENSY-2):
            Crc = CRC16(Crc, WriteDataMotor[i])
        CrcLByte = Crc & 0x00FF
        CrcHByte = (Crc & 0xFF00) >> 8 # Instead of /256, that value remains as an integer

        # Write the send data to a bytearray, 0..32, 33 Byte
        WriteDataMotor = [WriteByte0, WriteByte1, WriteByte2, WriteByte3, WriteByte4,
                          WriteByte5, WriteByte6, WriteByte7, WriteByte8, WriteByte9,
                          WriteByte10, WriteByte11, WriteByte12, WriteByte13, WriteByte14,
                          WriteByte15, WriteByte16, WriteByte17, WriteByte18, WriteByte19,
                          WriteByte20, WriteByte21, WriteByte22, WriteByte23, WriteByte24,
                          WriteByte25, WriteByte26, WriteByte27, WriteByte28, WriteByte29,
                          WriteByte30, CrcLByte, CrcHByte]

        GPIO.output(7, True) # RS485 Transmitter enable, high=send
        HWSERIAL.reset_output_buffer() # Clear output buffer
        NumberBytesWritten = HWSERIAL.write(WriteDataMotor) # Send bytearray
        time.sleep(0.05)
        GPIO.output(7, False) # RS485 Transmitter enable, low=receive

""" ######################################################################################## """
""" End loop forever """
GPIO.output(7, False)
GPIO.cleanup()


