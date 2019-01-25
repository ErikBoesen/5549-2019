import wpilib
import logging
from math import *
from wpilib.drive import DifferentialDrive
from wpilib import SmartDashboard
from networktables import NetworkTables
from ctre import *

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.frontRightMotor = WPI_TalonSRX(1)
        self.rearRightMotor = WPI_TalonSRX(2)
        self.frontLeftMotor = WPI_TalonSRX(3)
        self.rearLeftMotor = WPI_TalonSRX(4)

        self.right_encoder = self.rearRightMotor
        self.left_encoder = self.frontLeftMotor
        self.P = 0.1
        self.setpoint = 10000

        # defining motor groups
        self.left = wpilib.SpeedControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.right = wpilib.SpeedControllerGroup(self.frontRightMotor, self.rearRightMotor)

        # setting up drive group for drive motors
        self.drive = DifferentialDrive(self.left, self.right)
        self.drive.setExpiration(0.1)

        # object that handles basic intake operations
        self.omnom_left_motor = wpilib.Spark(7)
        self.omnom_right_motor = wpilib.Spark(8)

        # defining omnom motor groups
        self.omnom_left = wpilib.SpeedControllerGroup(self.omnom_left_motor)
        self.omnom_right = wpilib.SpeedControllerGroup(self.omnom_right_motor)

        # setting up omnom group for omnom motors
        self.omnom = DifferentialDrive(self.omnom_left, self.omnom_right)
        self.omnom.setExpiration(0.1)

        # Lift motors
        self.motorTopRight = wpilib.Victor(0)
        self.motorTopLeft = wpilib.Victor(1)
        self.motorBottomRight = wpilib.Victor(2)
        self.motorBottomLeft = wpilib.Victor(3)

        # lift motor groups
        self.lift_group = wpilib.SpeedControllerGroup(self.motorTopRight, self.motorTopLeft, self.motorBottomLeft, self.motorBottomRight)

        # joystick 0, 1, 2 on the driver station
        self.leftStick = wpilib.Joystick(0)
        self.rightStick = wpilib.Joystick(1)
        self.stick = wpilib.Joystick(2)

        logging.basicConfig(level=logging.DEBUG)
        self.sd = NetworkTables.getTable('SmartDashboard')
        NetworkTables.initialize(server='10.55.49.2')

        self.timer = wpilib.Timer()

        self.pdp = wpilib.PowerDistributionPanel()
        self.robocontroller = wpilib.RobotController()
        self.DS = wpilib.DriverStation.getInstance()
        self.timer = wpilib.Timer()

        # initialization of the HTTP camera
        wpilib.CameraServer.launch()

    def autonomousInit(self):
        self.frontLeftMotor.setQuadraturePosition(0, 0)
        self.rearRightMotor.setQuadraturePosition(0, 0)

    def autonomousPeriodic(self):
        def pid():
            self.rightPos = fabs(self.rearRightMotor.getQuadraturePosition())
            self.leftPos = fabs(self.frontLeftMotor.getQuadraturePosition())
            error = self.setpoint - ((self.leftPos + self.rightPos) / 2)
            self.rcw = self.P * error
            self.drive.arcadeDrive(0.3, self.rcw)

        def encoder_test():
            self.rightPos = fabs(self.rearRightMotor.getQuadraturePosition())
            self.leftPos = fabs(self.frontLeftMotor.getQuadraturePosition())
            self.distIn = (((self.leftPos + self.rightPos) / 2) / 4096) * 18.84955
            if 0 <= self.distIn <= 72:
                self.drive.tankDrive(0.5, 0.5)
            else:
                self.drive.tankDrive(0, 0)

        def auto_checkup():
            pass

        if self.DS.getGameSpecificMessage() == "RRR":
            encoder_test()
        elif self.DS.getGameSpecificMessage() == "ACT":
            auto_checkup()

    def teleopInit(self):
        self.drive.setSafetyEnabled(True)

        self.rearRightMotor.setQuadraturePosition(0, 0)
        self.frontLeftMotor.setQuadraturePosition(0, 0)

    def teleopPeriodic(self):
        # smart dashboard diagnostics
        self.sd.putString("", "Diagnostics")
        self.sd.putNumber("Temperature: ", self.pdp.getTemperature())
        self.sd.putNumber("Battery Voltage: ", self.robocontroller.getBatteryVoltage())
        self.sd.putBoolean(" Browned Out?", self.robocontroller.isBrownedOut())
        self.sd.putBoolean(" Autonomous?", self.DS.isAutonomous())
        self.sd.putBoolean(" FMS Connection", self.DS.isFMSAttached())
        self.sd.putNumber("Average Encoder Speed: ", self.rearRightMotor.getQuadratureVelocity())

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

        # lift controller mapping
        if self.stick.getRawAxis(3):
            self.lift_group.set(self.stick.getRawAxis(3) / 1.5)
        elif self.stick.getRawAxis(2):
            self.lift_group.set(self.stick.getRawAxis(2) / 40)
        elif self.stick.getRawButton(5):
            self.lift_group.set(self.stick.getRawButton(5) / 20)
        else:
            self.lift_group.set(0)

        left_omnom_stick = self.stick.getRawAxis(5) / 1.25
        right_omnom_stick = self.stick.getRawAxis(1) / 1.25

        # drives intake system using tank steering
        self.omnom.tankDrive(left_omnom_stick, right_omnom_stick)

        rightAxis = self.rightStick.getRawAxis(1)
        leftAxis = self.leftStick.getRawAxis(1)

        # drives drive system using tank steering
        self.drive.tankDrive(-leftAxis / 1.50, -rightAxis / 1.50)


if __name__ == '__main__':
    wpilib.run(MyRobot)