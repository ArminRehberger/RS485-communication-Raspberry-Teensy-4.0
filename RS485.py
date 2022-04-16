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
""" Global variables """
NumbersByteWritten = 0
ByteReceived = 0
WriteData = False

# Bytearray 0..22, 23 Byte receive from teensy
ReadDataMotor = bytearray([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, # 23 Byte read data
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0])

# Bytearray 0..29, 30 Byte send to teensy
WriteDataMotor = bytearray([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, # 30 Byte write data
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

# Write anything to send data, 30 Byte, Raspberry -> Teensy
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
WriteByte27Checksum = 92
WriteByte28CR = 13 # Carriage return
WriteByte29LF = 10 # Line feed

""" ######################################################################################## """
""" Start loop forever """
while True:

    """ ######################################################################################## """
    """ RS485 read data """
    # Read a line including \n
    # Teensy -> Raspberry, Byte 0..22, 23 Bytes + \r + \n = 25 Bytes
    # \r = carraige return (Dec 13), \n = linefeed (Dec 10)
    # Read timeout must be set in Serial, if \n is missing in the string
    # Return value is a string
    # In Teensy use println, \r + \n is added automatically
    line = HWSERIAL.readline() # Read a line including \n
    HWSERIAL.reset_input_buffer() # Clear input buffer
    if line:
        ByteReceived = len(line)
        if ByteReceived == 25 and line[23] == 13 and line[24] == 10:
            WriteData = True
            print(line)
            for i in range(0, 23): # Copy Bytes 0..22, 23 Bytes to receive array
                ReadDataMotor[i] = line[i]

    """ ######################################################################################## """
    """ RS485 write data """
    # Write a bytearray
    # Raspberry -> Teensy, Byte 0..29, 30 Byte
    # In Teensy use readBytesUntil
    if WriteData == True:
        WriteData = False
        
        # Write the send data to a bytearray, 0..29, 30 Byte
        WriteDataMotor = [WriteByte0, WriteByte1, WriteByte2, WriteByte3, WriteByte4,
                          WriteByte5, WriteByte6, WriteByte7, WriteByte8, WriteByte9,
                          WriteByte10, WriteByte11, WriteByte12, WriteByte13, WriteByte14,
                          WriteByte15, WriteByte16, WriteByte17, WriteByte18, WriteByte19,
                          WriteByte20, WriteByte21, WriteByte22, WriteByte23, WriteByte24,
                          WriteByte25, WriteByte26, WriteByte27Checksum, WriteByte28CR, WriteByte29LF]
        
        GPIO.output(7, True) # RS485 Transmitter enable, high=send
        HWSERIAL.reset_output_buffer() # Clear output buffer
        NumbersByteWritten = HWSERIAL.write(WriteDataMotor) # Send bytearray
        time.sleep(0.05)
        GPIO.output(7, False) # RS485 Transmitter enable, low=receive

""" ######################################################################################## """
""" End loop forever """
GPIO.output(7, False)
GPIO.cleanup()


