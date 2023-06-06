import os
import time
import numpy
import ADS1263
import RPi.GPIO as GPIO
import multiprocessing
import numpy as np
import signal

REF = 5.08          # Modify according to actual voltage

ADDR_TORQUE_ENABLE          = 64
ADDR_DRIVE_MODE             = 10
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 1 # Test

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
dxl_ids = [3, 4]


def signal_handler(signum, frame):
    # signal handler function to terminate child processes gracefully
    global terminateProgram
    terminateProgram = True
    print ("Terminating program")

def getADC(ADC, TOTAL_SAMPLES):

    recBuf = np.zeros((TOTAL_SAMPLES+1), dtype=np.float32)
    recBufIdx = int(0)
    print("Starting ADC")
    global terminateProgram
    while False == terminateProgram:
        recBuf[recBufIdx] = ADC.ADS1263_GetChannalValue(int(0))    # get ADC1 value
        recBufIdx += 1
        recBufIdx = recBufIdx % TOTAL_SAMPLES
    print ("ADC killed")
    return (recBuf, recBufIdx)
    
def main():
    signal.signal(signal.SIGINT, signal_handler)
    if os.name == 'nt':
        import msvcrt
        def getch():
            return msvcrt.getch().decode()
    else:
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        def getch():
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    global terminateProgram 
    terminateProgram = False

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler

    ADC = ADS1263.ADS1263()
    if (ADC.ADS1263_init_ADC1('ADS1263_14400SPS') == -1):
        exit()
    ADC.ADS1263_SetMode(0) # 0 is singleChannel, 1 is diffChannel

    channelList = [0, 1, 2, 3, 4]  # The channel must be less than 10

    SAMPLING_RATE = 14400
    DURATION = 10 #seconds
    TOTAL_SAMPLES = SAMPLING_RATE * DURATION

    (recBuf, recBufIdx) = getADC(ADC, TOTAL_SAMPLES)
    recBuf[-1] = recBufIdx # last index of the circular buffer
    np.savetxt("record.csv", recBuf, delimiter = ",")
    np.savetxt("idx.txt", recBufIdx)
        

if __name__ == "__main__":
    main()
    

