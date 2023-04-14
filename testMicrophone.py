import os
import time
import numpy
import ADS1263
import RPi.GPIO as GPIO
import multiprocessing
import numpy as np
import signal


REF = 5.08          # Modify according to actual voltage

from dynamixel_sdk import * # Uses Dynamixel SDK library

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

SAMPLING_RATE = 7200
DURATION = 20 #seconds
TOTAL_SAMPLES = SAMPLING_RATE * DURATION

recBuf = np.zeros((TOTAL_SAMPLES+1), dtype=np.float32)
timePlot = np.linspace(-20, 0, TOTAL_SAMPLES)
recBufIdx = int(0)

def signal_handler(signum, frame):
    # signal handler function to terminate child processes gracefully
    for process in multiprocessing.active_children():
        process.terminate()

def runMotors(packetHandler, getch, portHandler):
    while 1:
        print("Press any key to continue! (or press ESC to quit!)")
        if getch() == chr(0x1b):
            break

        currentId = int(input("Motor to control: "))
        position = int(input("Position to set: "))

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, currentId, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, currentId, ADDR_GOAL_POSITION, position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Position successfully sent")

        print("Press key to continue")
        getch()
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, currentId, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

def getADC(ADC, channelList):
    while 1:
        recBuf[recBufIdx] = ADC.ADS1263_GetChannalValue(int(0))    # get ADC1 value
        recBufIdx += 1
        recBufIdx = recBufIdx % TOTAL_SAMPLES

    
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
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    ADC = ADS1263.ADS1263()
    if (ADC.ADS1263_init_ADC1('ADS1263_7200SPS') == -1):
        exit()
    ADC.ADS1263_SetMode(0) # 0 is singleChannel, 1 is diffChannel

    channelList = [0, 1, 2, 3, 4]  # The channel must be less than 10

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()


    currentId = 1
    #p1 = multiprocessing.Process(target=runMotors, args=(packetHandler, getch, portHandler))
    p1 = multiprocessing.Process(target=getADC, args=(ADC, channelList))
    p1.start()
    #p2.start()
    runMotors(packetHandler, getch, portHandler)
    p1.join()
    #p2.join()
    recBuf[-1] = recBufIdx # last index of the circular buffer
    np.savetxt("record.csv", recBuf, delimiter = ",")
        

if __name__ == "__main__":
    main()
    

