import os
import time

import dynamixel_sdk # Uses Dynamixel SDK library

ADDR_TORQUE_ENABLE          = 64
ADDR_DRIVE_MODE             = 10
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600



class ServoJoint():
    #something


class ServoHand():
    '''
    This class handles the entire ServoHand object.
    The servo joints will be accessed as members of this class.
    '''
    mPortHandler = None
    mPacketHandler = None
    mDxlCommResult = None
    mDxlError = None
    '''
    ServoHand Constructor:
    Input Parameters:
    deviceName - string defining where the USB UART connection is.
    ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"    
    
    DYNAMIXEL Protocol Version (1.0 / 2.0)
    https://emanual.robotis.com/docs/en/dxl/protocol2/
    PROTOCOL_VERSION            = 2.0
    '''
    def __init__(self, deviceName='/dev/ttyUSB0', protoVer=2.0):
        self.mPortHandler = dynamixel_sdk.PortHandler(deviceName)
        self.mPacketHandler = dynamixel_sdk.PacketHandler(protoVer)
    
    def 


    def torque_enable(motorId):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, currentId, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")

def set_motor_position(motorId, degree):
    #something

    