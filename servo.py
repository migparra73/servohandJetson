import os
import time
import numpy

import dynamixel_sdk as Dynamixel # Uses Dynamixel SDK library

class Servo:
    portHandler = None
    packetHandler = None

    ADDR_TORQUE_ENABLE          = 64
    ADDR_DRIVE_MODE             = 10
    VELOCITY_DRIVE_MODE         = 4
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
    PROFILE_VELOCITY            = 112
    PROFILE_ACCELERATION        = 108

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

    def __init__(self, deviceName=DEVICENAME, protoVersion=PROTOCOL_VERSION, baudRate = BAUDRATE):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = Dynamixel.PortHandler(deviceName)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = Dynamixel.PacketHandler(protoVersion)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(baudRate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

    # Default is velocity drive mode.
    def setDriveMode(self, servoId, driveMode = VELOCITY_DRIVE_MODE):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, servoId, self.ADDR_DRIVE_MODE, driveMode)

    def enableServo(self, servoId):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, servoId, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")

    def disableServo(self, servoId):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, servoId, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")


    def setServoAngle(self, servoId, angle):
        # Convert Angle to range within the motor's limits. Right now this is 1024-2048 for one group,
        # and 
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, servoId, self.ADDR_GOAL_POSITION, angle)
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Position successfully sent")

    # Set the Profile Velocity parameter - use this to set the velocity of the next movement.
    def setProfileVelocity(self, servoId, profileVelocity):
        # Write the Profile Velocity value to the corresponding register
        self.packetHandler.write2ByteTxRx(self.portHandler, servoId, self.PROFILE_VELOCITY, profileVelocity)

    def setProfileAcceleration(self, servoId, profileAcceleration):
        # Write the Profile Acceleration value to the corresponding register
        self.packetHandler.write2ByteTxRx(self.portHandler, servoId, self.PROFILE_ACCELERATION, profileAcceleration)
