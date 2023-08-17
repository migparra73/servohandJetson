import os
import time
import numpy

import dynamixel_sdk as Dynamixel # Uses Dynamixel SDK library

import busio
import board
from adafruit_servokit import ServoKit

class Servo:
    portHandler = None
    packetHandler = None
    groupSyncWritePosition = None
    groupSyncWriteVelocity = None
    groupSyncReadPosition = None
    groupSyncReadVelocity = None

    i2c_bus1 = None
    adafruitKit = None

    sclPort = None
    sdaPort = None


    ADDR_TORQUE_ENABLE          = 64
    ADDR_DRIVE_MODE             = 10
    VELOCITY_DRIVE_MODE         = 0
    TIME_DRIVE_MODE             = 4
    NORMAL_ROTATION_DIRECTION   = 0
    REVERSE_ROTATION_DIRECTION  = 1
    
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    ADDR_PRESENT_VELOCITY       = 128
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
    ADDR_PROFILE_VELOCITY            = 112
    ADDR_PROFILE_ACCELERATION        = 108

    FACTORY_RESET_OPTION        = 0x02          # 0xFF : reset all values
                                                # 0x01 : reset all values except ID
                                                # 0x02 : reset all values except ID and baudrate

    SLIDER_MINIMUM_PWM_PULSE    = 1000
    SLIDER_MAXIMUM_PWM_PULSE    = 2000


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

    def __init__(self, deviceName=DEVICENAME, protoVersion=PROTOCOL_VERSION, baudRate = BAUDRATE, sclPort = board.SCL, sdaPort = board.SDA):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = Dynamixel.PortHandler(deviceName)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = Dynamixel.PacketHandler(protoVersion)
        
        # Initialize GroupSyncWrite instances
        self.groupSyncWritePosition = Dynamixel.GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, 4)     
        self.groupSyncWriteVelocity = Dynamixel.GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_PROFILE_VELOCITY, 4)

        # Initialize GroupSyncWrite instances
        self.groupSyncReadPosition = Dynamixel.GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_POSITION, 4)     
        self.groupSyncReadVelocity = Dynamixel.GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_VELOCITY, 4)

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

        # On the Jetson Nano
        # Bus 0 (pins 28,27) is board SCL_1, SDA_1 in the jetson board definition file
        # Bus 1 (pins 5, 3) is board SCL, SDA in the jetson definition file
        # Default is to Bus 1; We are using Bus 0, so we need to construct the busio first ...
        print("Initializing Servos")
        self.sclPort = sclPort
        self.sdaPort = sdaPort
        self.i2c_bus1=(busio.I2C(self.sclPort, self.sdaPort))
        print("Initializing ServoKit")
        self.adafruitKit = ServoKit(channels=16, i2c=self.i2c_bus1)
        self.adafruitKit.continuous_servo[0].set_pulse_width_range(self.SLIDER_MINIMUM_PWM_PULSE, self.SLIDER_MAXIMUM_PWM_PULSE)
        print("Slider initialized.")

    # Default is velocity drive mode.
    def setDriveMode(self, servoId, driveMode = VELOCITY_DRIVE_MODE):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, servoId, self.ADDR_DRIVE_MODE, driveMode)
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Drive mode has been successfully set")

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

    def setServoAngleMultiple(self, servoIds, angles):
        assert(len(servoIds) == len(angles))
        for idx, servoId in enumerate(servoIds):
            dxl_addparam_result = self.groupSyncWritePosition.addParam(servoId, [Dynamixel.DXL_LOBYTE(Dynamixel.DXL_LOWORD(angles[idx])), Dynamixel.DXL_HIBYTE(Dynamixel.DXL_LOWORD(angles[idx])), \
                                                                                 Dynamixel.DXL_LOBYTE(Dynamixel.DXL_HIWORD(angles[idx])), Dynamixel.DXL_HIBYTE(Dynamixel.DXL_HIWORD(angles[idx]))])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed, retry" % idx)
                return False
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWritePosition.txPacket()
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        self.groupSyncWritePosition.clearParam()
        return True

    def getServoAngleMultiple(self, servoIds):
        # Syncread present position
        allReadsIn = None
        returnedDataArray = [-1] * len(servoIds)
        retries = 0
        
        for servoId in servoIds:
            dxl_addparam_result = self.groupSyncReadPosition.addParam(servoId)
            if dxl_addparam_result != True:
                print("Failed to add id %d to be read" % servoId)
            else:
                print("Adding servo %d to be read from " % servoId)
        
        dxl_comm_result = self.groupSyncReadPosition.txRxPacket()
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        for idx, servoId in enumerate(servoIds):
            dxl_getdata_result = self.groupSyncReadPosition.isAvailable(servoId, self.ADDR_PRESENT_POSITION, 4)
            if dxl_getdata_result == True:
                returnedDataArray[idx] = self.groupSyncReadPosition.getData(servoId, self.ADDR_GOAL_POSITION, 4)
            else:
                print("Servo getServoAngleMultiple: Something is wrong")
        self.groupSyncReadPosition.clearParam()
        return returnedDataArray

    def setServoVelocityMultiple(self, servoIds, velocities):
        assert(len(servoIds) == len(velocities))
        for idx, servoId in enumerate(servoIds):
            dxl_addparam_result = self.groupSyncWriteVelocity.addParam(servoId, [Dynamixel.DXL_LOBYTE(Dynamixel.DXL_LOWORD(velocities[idx])), Dynamixel.DXL_HIBYTE(Dynamixel.DXL_LOWORD(velocities[idx])), \
                                                                                 Dynamixel.DXL_LOBYTE(Dynamixel.DXL_HIWORD(velocities[idx])), Dynamixel.DXL_HIBYTE(Dynamixel.DXL_HIWORD(velocities[idx]))])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed, retry" % idx)
                return False
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWriteVelocity.txPacket()
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        self.groupSyncWriteVelocity.clearParam()
        return True


    def getServoVelocityMultiple(self, servoIds):
        # Syncread present position
        returnedDataArray = [None] * len(servoIds)
        for servoId in servoIds:
            dxl_addparam_result = self.groupSyncReadVelocity.addParam(servoId)
            if dxl_addparam_result != True:
                print("Failed to add id %d to be read" % servoId)
        
        dxl_comm_result = self.groupSyncReadVelocity.txRxPacket()
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        for idx, servoId in enumerate(servoIds):
            dxl_getdata_result = self.groupSyncReadVelocity.isAvailable(servoId, self.ADDR_PRESENT_VELOCITY, 4)
            if dxl_getdata_result == True:
                returnedDataArray[idx] = self.groupSyncReadVelocity.getData(servoId, self.ADDR_PRESENT_VELOCITY, 4)
            else:
                print("Somthing is wrong - get servo velocity multiple")
        self.groupSyncReadPosition.clearParam()
        return returnedDataArray

    # Set the Profile Velocity parameter - use this to set the velocity of the next movement.
    def setProfileVelocity(self, servoId, profileVelocity):
        # Write the Profile Velocity value to the corresponding register
        
        # profileVelocity is in degrees per second.
        # Convert this to units of 0.229 revolutions / min
        profVel = int(profileVelocity / 1.374)
        if profVel < 0:
            profVel=0
        print("%d" % profVel)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, servoId, self.PROFILE_VELOCITY, profileVelocity)
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def getProfileVelocity(self, servoId):
        profileVelocity, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, servoId, self.PROFILE_VELOCITY)
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return profileVelocity



    def setProfileAcceleration(self, servoId, profileAcceleration):
        # Raw value input.
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, servoId, self.PROFILE_ACCELERATION, profileAcceleration)
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Acceleration has been successfully set")

    def rebootServo(self, servoId):
        dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, servoId)
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("Successfully rebooted")

    
    def getErrorStatus(self, servoId):
        errorStatus, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, servoId, 70)
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("HW Error Status = %x" % errorStatus)

    def factoryReset(self, servoId):
        dxl_comm_result, dxl_error = self.packetHandler.factoryReset(self.portHandler, servoId, self.FACTORY_RESET_OPTION)
        if dxl_comm_result != Dynamixel.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("Successfully factory reset")

    def setSliderPosition(self, positionRatio):
        self.adafruitKit.continuous_servo[0].fraction = positionRatio