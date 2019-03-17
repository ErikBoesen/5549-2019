'''
Destination: Deep Space 2019 - GEMINI from Gryphon Robotics
'''

import wpilib
import logging
from math import *
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
from ctre import *
from robotpy_ext.control.toggle import Toggle


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        ''' Initialization of robot objects. '''

        ''' Talon SRX Initialization '''
        # drive train motors
        self.frontRightMotor = WPI_TalonSRX(4)
        self.rearRightMotor = WPI_TalonSRX(3)
        self.frontLeftMotor = WPI_TalonSRX(1)
        self.rearLeftMotor = WPI_TalonSRX(2)

        ''' Encoders '''
        # drive train encoders
        self.rightEncoder = self.frontRightMotor
        self.leftEncoder = self.frontLeftMotor

        # lift encoder
        self.liftEncoder = wpilib.Encoder(8, 9)

        # liftArm encoder
        self.liftArmEncoder = wpilib.Encoder(5, 6, True)

        ''' Motor Groups '''
        # drive train motor groups
        self.left = wpilib.SpeedControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.right = wpilib.SpeedControllerGroup(self.frontRightMotor, self.rearRightMotor)

        # drive train drive group
        self.drive = DifferentialDrive(self.left, self.right)
        self.drive.setExpiration(0.1)

        ''' Victor SPX Initialization '''
        # lift motors
        self.liftOne = WPI_VictorSPX(1)
        self.liftTwo = WPI_VictorSPX(2)
        self.lift = wpilib.SpeedControllerGroup(self.liftOne, self.liftTwo)

        # lift arm motors
        self.liftArmOne = WPI_VictorSPX(3)
        self.liftArmTwo = WPI_VictorSPX(4)
        self.liftArm = wpilib.SpeedControllerGroup(self.liftArmOne, self.liftArmTwo)

        # cargo intake motor
        self.cargo = WPI_VictorSPX(5)

        ''' Controller Initialization and Mapping '''
        # joystick - 0, 1 | controller - 2
        self.joystick = wpilib.Joystick(1)
        self.xbox = wpilib.Joystick(2)
        self.buttonBox = wpilib.Joystick(3)

        ''' Button Status '''
        self.buttonStatusOne = Toggle(self.xbox, 8)
        self.buttonStatusTwo = Toggle(self.buttonBox, 2)
        self.buttonStatusThree = Toggle(self.xbox, 7)
        self.buttonStatusFour = Toggle(self.buttonBox, 9)
        self.buttonStatusFive = Toggle(self.buttonBox, 8)
        self.buttonStatusSix = Toggle(self.buttonBox, 7)
        self.buttonStatusSeven = Toggle(self.buttonBox, 6)


        ''' Pneumatic Initialization '''
        self.Compressor = wpilib.Compressor(0)
        self.Compressor.setClosedLoopControl(True)
        self.enable = self.Compressor.getPressureSwitchValue()
        self.DoubleSolenoidOne = wpilib.DoubleSolenoid(0, 1)    # gear shifting
        self.DoubleSolenoidTwo = wpilib.DoubleSolenoid(2, 3)    # hatch panel claw
        self.DoubleSolenoidThree = wpilib.DoubleSolenoid(4, 5)  # hatch panel ejection
        self.Compressor.start()

        ''' Smart Dashboard '''
        # connection for logging & Smart Dashboard
        logging.basicConfig(level=logging.DEBUG)
        self.sd = NetworkTables.getTable('SmartDashboard')
        NetworkTables.initialize(server='10.55.49.2')
        self.sd.putString("  ", "Connection")

        # Smart Dashboard classes
        self.PDP = wpilib.PowerDistributionPanel()
        self.roboController = wpilib.RobotController()
        self.DS = wpilib.DriverStation.getInstance()

        ''' Sensors '''
        # Hall Effect Sensor
        self.minHall = wpilib.DigitalInput(7)
        self.maxHall = wpilib.DigitalInput(4)
        self.limitSwitch = wpilib.DigitalInput(3)
        self.ultrasonic = wpilib.AnalogInput(2)
        self.cargoUltrasonic = wpilib.AnalogInput(3)

        ''' Timer '''
        self.timer = wpilib.Timer()

        ''' Camera '''
        # initialization of the HTTP camera
        wpilib.CameraServer.launch('vision.py:main')
        self.sd.putString("", "Top Camera")
        self.sd.putString(" ", "Bottom Camera")

    def autonomousInit(self):
        ''' Executed each time the robot enters autonomous. '''

        # timer config
        self.timer.reset()
        self.timer.start()

        # drive train encoder reset
        self.rightEncoder.setQuadraturePosition(0, 0)
        self.leftEncoder.setQuadraturePosition(0, 0)

        self.liftEncoder.reset()
        self.liftArmEncoder.reset()

    def autonomousPeriodic(self):

        self.sd.putBoolean("LIFT RESET ", self.minHall.get())

        ''' Called periodically during autonomous. '''

        def fourbar():
            if self.limitSwitch.get() is True:
                self.liftArm.set(0.25)
            else:
                self.liftArm.set(0)

        '''Test Methods'''
        def encoder_test():
            ''' Drives robot set encoder distance away '''
            self.rightPos = fabs(self.rightEncoder.getQuadraturePosition())
            self.leftPos = fabs(self.leftEncoder.getQuadraturePosition())
            self.distIn = (((self.leftPos + self.rightPos) / 2) / 4096) * 18.84955
            if 0 <= self.distIn <= 72:
                self.drive.tankDrive(0.5, 0.5)
            else:
                self.drive.tankDrive(0, 0)

        def Diagnostics():
            ''' Smart Dashboard Tests'''
            self.sd.putNumber("Temperature: ", self.PDP.getTemperature())
            self.sd.putNumber("Battery Voltage: ", self.roboController.getBatteryVoltage())
            self.sd.putBoolean(" Browned Out?", self.roboController.isBrownedOut)

            # Smart Dashboard diagnostics
            self.sd.putNumber("Right Encoder Speed: ", abs(self.rightEncoder.getQuadratureVelocity()))
            self.sd.putNumber("Left Encoder Speed: ", abs(self.leftEncoder.getQuadratureVelocity()))
            self.sd.putNumber("Lift Encoder: ", self.liftEncoder.getDistance())

        def Pressure():
            self.Compressor.start()

        def cargoOne():
            if self.liftEncoder.get() < 133:  # Cargo 1
                self.lift.set(0.5)
            elif self.liftEncoder.get() >= 133:
                self.lift.set(0.05)
                self.buttonStatusOne = False

        def cargoTwo():
            if self.liftEncoder.get() < 270:   # Cargo 2
                self.lift.set(0.5)
            elif self.liftEncoder.get() >= 270:
                self.lift.set(0.05)
                self.buttonStatusTwo = False

        def cargoThree():
            if self.liftEncoder.get() < 415:   # Cargo 3
                self.lift.set(0.5)
            elif self.liftEncoder.get() >= 415:
                self.lift.set(0.05)
                self.buttonStatusThree = False

        def hatchOne():
            if self.liftEncoder.getDistance() < 125:  # Hatch 2
                self.lift.set(0.3)
            elif self.liftEncoder.getDistance() >= 125:
                self.lift.set(0.05)

        def hatchTwo():
            if self.liftEncoder.getDistance() < 305:   # Hatch 2
                self.lift.set(0.5)
            elif self.liftEncoder.getDistance() >= 305:
                self.lift.set(0.05)

        def hatchThree():
            if self.liftEncoder.get() < 378:   # Hatch 3
                self.lift.set(0.5)
            elif self.liftEncoder.get() >= 378:
                self.lift.set(0.05)
                self.buttonStatusSix = False

        def liftEncoderReset():
            if self.limitSwitch.get() is True:
                self.liftArm.set(-0.2)
            elif self.limitSwitch.get() is False:
                self.liftArm.set(0)
                self.liftArmEncoder.reset()

            if self.minHall.get() is True:
                self.lift.set(0)
            elif self.minHall.get() is False:
                self.lift.set(0)
                self.liftEncoder.reset()

        def fourbarreset():
            if self.limitSwitch.get() is True:
                self.liftArm.set(0.4)
            elif self.limitSwitch.get() is False:
                self.liftArm.set(0)
                self.liftArmEncoder.reset()

        ''' Button Box Level Mapping '''
        # if self.buttonStatusOne.on:
        #     cargoThree()
        #     self.allButtonStatus[0] = True
        # elif self.buttonStatusOne.off:
        #     liftEncoderReset()
        #     self.allButtonStatus[0] = False
        #
        # if self.buttonStatusTwo.on:
        #     hatchThree()
        # elif self.buttonStatusTwo.off:
        #     liftEncoderReset()
        #
        # if self.buttonStatusThree.on:
        #     cargoTwo()
        # elif self.buttonStatusThree.off:
        #     liftEncoderReset()
        #
        # if self.joystick.getRawButton(12):
        #     hatchTwo()
        # elif self.buttonStatusFour.off:
        #     liftEncoderReset()
        #
        # elif self.buttonStatusFive.on:
        #    cargoOne()
        # elif self.buttonStatusFive.off:
        #     liftEncoderReset()
        #
        # if self.buttonStatusSix.on:
        #    hatchOne()
        # elif self.buttonStatusSix.off:
        #     liftEncoderReset()
        #
        # elif self.buttonStatusSeven.on:
        #      liftEncoderReset()

        if self.buttonStatusOne.on:
            hatchOne()
        elif self.buttonStatusThree.on:
            hatchTwo()

        if self.minHall.get() is False:
            self.liftEncoder.reset()

        if self.limitSwitch.get() is False:
            self.liftArmEncoder.reset()

        ''' Test Execution '''
        if self.DS.getGameSpecificMessage() == "pressure":
            Pressure()
        elif self.DS.getGameSpecificMessage() == "diagnostics":
            Diagnostics()
        elif self.DS.getGameSpecificMessage() == "fourbar":
            fourbar()

        ''' Smart Dashboard '''
        # compressor state
        if self.Compressor.enabled() is True:
            self.sd.putString("Compressor Status: ", "Enabled")
        elif self.Compressor.enabled() is False:
            self.sd.putString("Compressor Status: ", "Disabled")

        ''' Pneumatics Dashboard States '''
        # gear state
        if self.DoubleSolenoidOne.get() == 1:
            self.sd.putString("Gear Shift: ", "HIGH SPEED!!!")
        elif self.DoubleSolenoidOne.get() == 2:
            self.sd.putString("Gear Shift: ", "Low")

        # ejector state
        if self.DoubleSolenoidThree.get() == 2:
            self.sd.putString("Ejector Pins: ", "Ejected")
        elif self.DoubleSolenoidThree.get() == 1:
            self.sd.putString("Ejector Pins: ", "Retracted")

        # claw state
        if self.DoubleSolenoidTwo.get() == 2:
            self.sd.putString("Claw: ", "Open")
        elif self.DoubleSolenoidTwo.get() == 1:
            self.sd.putString("Claw: ", "Closed")

        ''' Ultrasonic Range Detection '''
        # robot ultrasonic
        self.ultraValue = self.ultrasonic.getVoltage()
        if 0.142 <= self.ultraValue <= 0.146:
            self.sd.putString("PLAYER STATION RANGE: ", "YES!!!!")
        else:
            self.sd.putString("PLAYER STATION RANGE: ", "NO!")

        #self.sd.putNumber("Ultrasonic Voltage: ", self.ultraValue)

        # cargo ultrasonic
        self.cargoUltraValue = self.cargoUltrasonic.getVoltage()
        if 0.70 <= self.cargoUltraValue <= 1.56:
            self.sd.putString("HATCH RANGE: ", "HATCH IN RANGE")
        else:
            self.sd.putString("HATCH RANGE: ", "NOT IN RANGE")

        ''' Pneumatics Control '''
        # compressor
        if self.xbox.getRawButton(9):
            self.Compressor.stop()
        elif self.xbox.getRawButton(10):
            self.Compressor.start()
        elif self.joystick.getRawButton(1):  # shift right
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.joystick.getRawButton(2):  # shift left
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kReverse)
        elif self.xbox.getRawButton(3):  # open claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.xbox.getRawButton(2):  # close claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kReverse)
        elif self.xbox.getRawButton(4):  # eject
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.xbox.getRawButton(1):  # retract
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kReverse)

        ''' Victor SPX (Lift, Lift Arm, Cargo) '''
        # lift control
        if self.buttonStatusThree.off and self.buttonStatusOne.off:
            if self.xbox.getRawButton(5):  # hold
                self.lift.set(0.05)
            elif self.xbox.getRawAxis(3):  # up
                self.lift.set(self.xbox.getRawAxis(3) * 0.65)
            elif self.xbox.getRawAxis(2):  # down
                self.lift.set(-self.xbox.getRawAxis(2) * 0.25)
            else:
                self.lift.set(0)

        # four-bar control
        if self.xbox.getRawButton(6):   # hold
            self.liftArm.set(0.10)
        elif not self.xbox.getRawButton(6):
            self.liftArm.set(-self.xbox.getRawAxis(1) * 0.25)
        else:
            self.liftArm.set(0)

        # cargo intake control
        # if self.xbox.getRawButton(7):   # hold
        #     self.cargo.set(0.12)
        # elif self.xbox.getRawAxis(5):  # take in
        #     self.cargo.set(self.xbox.getRawAxis(5) * 0.75)

        # controller mapping for tank steering
        self.driveAxis = self.joystick.getRawAxis(1)
        self.rotateAxis = self.joystick.getRawAxis(2)

        # drives drive system using tank steering
        if self.DoubleSolenoidOne.get() == 1:  # if on high gear
            self.divisor = 1.0  # 90% of high speed
            self.turnDivisor = 0.8
        elif self.DoubleSolenoidOne.get() == 2:  # if on low gear
            self.divisor = 0.85  # normal slow speed
            self.turnDivisor = 0.75
        else:
            self.divisor = 1.0

        if self.driveAxis != 0:
            self.leftSign = self.driveAxis / fabs(self.driveAxis)
        else:
            self.leftSign = 0

        if self.rotateAxis != 0:
            self.rightSign = self.rotateAxis / fabs(self.rotateAxis)
        else:
            self.rightSign = 0

        self.drive.arcadeDrive(-self.driveAxis * self.divisor, self.rotateAxis * 0.75)

    def teleopInit(self):
        ''' Executed at the start of teleop mode. '''

        self.drive.setSafetyEnabled(True)

        # drive train encoder reset
        self.rightEncoder.setQuadraturePosition(0, 0)
        self.leftEncoder.setQuadraturePosition(0, 0)

        # lift encoder rest
        self.liftEncoder.reset()

        # compressor
        self.Compressor.start()

    def teleopPeriodic(self):
        ''' Periodically executes methods during the teleop mode. '''
        '''        
        self.sd.putString(" ", "Match Info")
        self.sd.putString("Event Name: ", self.DS.getEventName())
        self.sd.putNumber("Match Time: ", self.timer.getMatchTime())
        self.sd.putNumber("Match Number: ", self.DS.getMatchTime())
        self.sd.putNumber("Location: ", self.DS.getLocation())
        if self.DS.getMatchType() == 3:
            self.sd.putString("Match Type: ", "Elimination")
        elif self.DS.getMatchType() == 1:
            self.sd.putString("Match Type: ", "Practice")
        elif self.DS.getMatchType() == 2:
            self.sd.putString("Match Type: ", "Qualification")
        else:
            self.sd.putString("Match Type: ", "None")

        if self.DS.getAlliance() == 0:
            self.sd.putString("Alliance: ", "Red")
        elif self.DS.getAlliance() == 1:
            self.sd.putString("Alliance: ", "Blue")
        else:
            self.sd.putString("Alliance: ", "Invalid")
        '''


        self.sd.putBoolean("LIFT RESET ", self.minHall.get())

        def cargoOne():
            if self.liftEncoder.get() < 133:  # Cargo 1
                self.lift.set(0.5)
            elif self.liftEncoder.get() >= 133:
                self.lift.set(0.05)
                self.buttonStatusOne = False

        def cargoTwo():
            if self.liftEncoder.get() < 270:   # Cargo 2
                self.lift.set(0.5)
            elif self.liftEncoder.get() >= 270:
                self.lift.set(0.05)
                self.buttonStatusTwo = False

        def cargoThree():
            if self.liftEncoder.get() < 415:   # Cargo 3
                self.lift.set(0.5)
            elif self.liftEncoder.get() >= 415:
                self.lift.set(0.05)
                self.buttonStatusThree = False

        def hatchOne():
            if self.liftEncoder.getDistance() < 125:  # Hatch 2
                self.lift.set(0.3)
            elif self.liftEncoder.getDistance() >= 125:
                self.lift.set(0.05)

        def hatchTwo():
            if self.liftEncoder.getDistance() < 305:   # Hatch 2
                self.lift.set(0.5)
            elif self.liftEncoder.getDistance() >= 305:
                self.lift.set(0.05)

        def hatchThree():
            if self.liftEncoder.get() < 378:   # Hatch 3
                self.lift.set(0.5)
            elif self.liftEncoder.get() >= 378:
                self.lift.set(0.05)
                self.buttonStatusSix = False

        def liftEncoderReset():
            if self.limitSwitch.get() is True:
                self.liftArm.set(-0.2)
            elif self.limitSwitch.get() is False:
                self.liftArm.set(0)
                self.liftArmEncoder.reset()

            if self.minHall.get() is True:
                self.lift.set(0)
            elif self.minHall.get() is False:
                self.lift.set(0)
                self.liftEncoder.reset()

        def fourbarreset():
            if self.limitSwitch.get() is True:
                self.liftArm.set(0.4)
            elif self.limitSwitch.get() is False:
                self.liftArm.set(0)
                self.liftArmEncoder.reset()

        ''' Button Box Level Mapping '''
        # if self.buttonStatusOne.on:
        #     cargoThree()
        #     self.allButtonStatus[0] = True
        # elif self.buttonStatusOne.off:
        #     liftEncoderReset()
        #     self.allButtonStatus[0] = False
        #
        # if self.buttonStatusTwo.on:
        #     hatchThree()
        # elif self.buttonStatusTwo.off:
        #     liftEncoderReset()
        #
        # if self.buttonStatusThree.on:
        #     cargoTwo()
        # elif self.buttonStatusThree.off:
        #     liftEncoderReset()
        #
        # if self.joystick.getRawButton(12):
        #     hatchTwo()
        # elif self.buttonStatusFour.off:
        #     liftEncoderReset()
        #
        # elif self.buttonStatusFive.on:
        #    cargoOne()
        # elif self.buttonStatusFive.off:
        #     liftEncoderReset()
        #
        # if self.buttonStatusSix.on:
        #    hatchOne()
        # elif self.buttonStatusSix.off:
        #     liftEncoderReset()
        #
        # elif self.buttonStatusSeven.on:
        #      liftEncoderReset()

        if self.buttonStatusOne.on:
            hatchOne()
        elif self.buttonStatusThree.on:
            hatchTwo()

        if self.minHall.get() is False:
            self.liftEncoder.reset()

        if self.limitSwitch.get() is False:
            self.liftArmEncoder.reset()

        ''' Smart Dashboard '''
        # compressor state
        if self.Compressor.enabled() is True:
            self.sd.putString("Compressor Status: ", "Enabled")
        elif self.Compressor.enabled() is False:
            self.sd.putString("Compressor Status: ", "Disabled")

        ''' Pneumatics Dashboard States '''
        # gear state
        if self.DoubleSolenoidOne.get() == 1:
            self.sd.putString("Gear Shift: ", "HIGH SPEED!!!")
        elif self.DoubleSolenoidOne.get() == 2:
            self.sd.putString("Gear Shift: ", "Low")

        # ejector state
        if self.DoubleSolenoidThree.get() == 2:
            self.sd.putString("Ejector Pins: ", "Ejected")
        elif self.DoubleSolenoidThree.get() == 1:
            self.sd.putString("Ejector Pins: ", "Retracted")

        # claw state
        if self.DoubleSolenoidTwo.get() == 2:
            self.sd.putString("Claw: ", "Open")
        elif self.DoubleSolenoidTwo.get() == 1:
            self.sd.putString("Claw: ", "Closed")

        ''' Ultrasonic Range Detection '''
        # robot ultrasonic
        self.ultraValue = self.ultrasonic.getVoltage()
        if 0.142 <= self.ultraValue <= 0.146:
            self.sd.putString("PLAYER STATION RANGE: ", "YES!!!!")
        else:
            self.sd.putString("PLAYER STATION RANGE: ", "NO!")

        #self.sd.putNumber("Ultrasonic Voltage: ", self.ultraValue)

        # cargo ultrasonic
        self.cargoUltraValue = self.cargoUltrasonic.getVoltage()
        if 0.70 <= self.cargoUltraValue <= 1.56:
            self.sd.putString("HATCH RANGE: ", "HATCH IN RANGE")
        else:
            self.sd.putString("HATCH RANGE: ", "NOT IN RANGE")

        ''' Pneumatics Control '''
        # compressor
        if self.xbox.getRawButton(9):
            self.Compressor.stop()
        elif self.xbox.getRawButton(10):
            self.Compressor.start()
        elif self.joystick.getRawButton(1):  # shift right
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.joystick.getRawButton(2):  # shift left
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kReverse)
        elif self.xbox.getRawButton(3):  # open claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.xbox.getRawButton(2):  # close claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kReverse)
        elif self.xbox.getRawButton(4):  # eject
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.xbox.getRawButton(1):  # retract
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kReverse)

        ''' Victor SPX (Lift, Lift Arm, Cargo) '''
        # lift control
        if self.buttonStatusThree.off and self.buttonStatusOne.off:
            if self.xbox.getRawButton(5):  # hold
                self.lift.set(0.05)
            elif self.xbox.getRawAxis(3):  # up
                self.lift.set(self.xbox.getRawAxis(3) * 0.65)
            elif self.xbox.getRawAxis(2):  # down
                self.lift.set(-self.xbox.getRawAxis(2) * 0.25)
            else:
                self.lift.set(0)

        # four-bar control
        if self.xbox.getRawButton(6):   # hold
            self.liftArm.set(0.10)
        elif not self.xbox.getRawButton(6):
            self.liftArm.set(-self.xbox.getRawAxis(1) * 0.25)
        else:
            self.liftArm.set(0)

        # cargo intake control
        # if self.xbox.getRawButton(7):   # hold
        #     self.cargo.set(0.12)
        # elif self.xbox.getRawAxis(5):  # take in
        #     self.cargo.set(self.xbox.getRawAxis(5) * 0.75)

        # controller mapping for tank steering
        self.driveAxis = self.joystick.getRawAxis(1)
        self.rotateAxis = self.joystick.getRawAxis(2)

        # drives drive system using tank steering
        if self.DoubleSolenoidOne.get() == 1:  # if on high gear
            self.divisor = 1.0  # 90% of high speed
            self.turnDivisor = 0.8
        elif self.DoubleSolenoidOne.get() == 2:  # if on low gear
            self.divisor = 0.85  # normal slow speed
            self.turnDivisor = 0.75
        else:
            self.divisor = 1.0

        if self.driveAxis != 0:
            self.leftSign = self.driveAxis / fabs(self.driveAxis)
        else:
            self.leftSign = 0

        if self.rotateAxis != 0:
            self.rightSign = self.rotateAxis / fabs(self.rotateAxis)
        else:
            self.rightSign = 0

        self.drive.arcadeDrive(-self.driveAxis * self.divisor, self.rotateAxis * 0.75)


if __name__ == '__main__':
    wpilib.run(MyRobot)