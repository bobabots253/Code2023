package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Units;

/*  Robot Specs:
    4 TalonFX motors
    4-6 NEO motors
    2 TalonSRX motors
*/
//Hello
public class Constants {
    public static final double dt = 0.02;
    public static final double kMaxVoltage = 12.0;

    public static class InputPorts {
        public static final int driverController = 0, operatorController = 1;
    }

    public static class AutoConstants {
        
        public static final double[] DXMConstraints = {1, 0.5}, TXDConstraints = {480, 360};
        public static final double hubXOffset = 5, distToCargo = 5;
    }

    public static class DriverConstants {
        /* Common drive mode settings */
        public static final double kJoystickDeadband = 0.07; // How much of joystick is "dead" zone [0,1]
        public static final double kDriveSens = 0.5; // Overall speed setting (turn down for demos) [0,1]
        public static final double kTurnInPlaceSens = 0.3; // Maximum turn-in-place rate (in percent of max) to allow
                                                            // robot to turn to [0,1]
        public static final double kTurnSens = 1.0; // Maximum normal turning rate (in percent of max) to allow robot to
                                                  // turn to [0,1]
    }
    
    public static class ArmConstants {
        public static final int actuateMotorL = 4; //change to actuatemotor
        public static final int actuateMotorR = 0;
        /* PID Constants */
        public static double kP = 2.9;
        public static double kI = 0;
        public static double kD = 0;

        /* Feedforward Constants */
        public static double kS = 0.402;
        public static double kCos = 0.771;
        public static double kV = 0.758;
        public static double kA = 0.00717;

        /* Intake constants */
        public static double kMaxVelocity = 0.25; // Maximum velocity to turn arm at, radians per second
        public static double kMaxAcceleration = 2; // Maximum acceleration to turn arm at, radians per second per second
        public static double kArmOffset = Math.toRadians(27); //

        public static edu.wpi.first.math.trajectory.TrapezoidProfile.State kStartRads;

        public static State autoDisplacementRads;
        public static int gearRatio = 25;

        //arm encoder values
        public static final double kCubeFloorIntakePosition = -5.285;
        public static final double kCubeMidScorePosition = -21.143;
        public static final double kCubeHighScorePosition = -27.286;
        public static final double kConeFloorUprightIntakePosition = -2.071;
        public static final double kConeFloorSidewaysIntakePosition = -1.786;
        public static final double kConeMidUprightScorePosition = -25.786;
        public static final double kConeMidSidewaysScorePosition = -17.286;
        public static final double kConeHighUprightScorePosition = -27.143;


    }

    public static class WristConstants {
        public static final int intakeMotor = 4;
        public static final int wristMotor = 10;
        public static final double gearRatio = 100.0;
        public static double autoDisplacementRads;

        public static final double kMaxVelocity = 2*Math.PI;
        public static final double kMaxAcceleration = 2*Math.PI;

        //wrist encoder values for intake
        public static final double kCubeFloorIntakePosition = 39.166;
        public static final double kCubeMidScorePosition = 51.262;
        public static final double kCubeHighScorePosition = 37.833;
        public static final double kConeFloorUprightIntakePosition = 23.524;
        public static final double kConeFloorSidewaysIntakePosition = 35.428;
        public static final double kConeMidUprightScorePosition = 57.763;
        public static final double kConeMidSidewaysScorePosition = 20.167;
        public static final double kConeHighUprightScorePosition = 47.166;

        /* PID Constants */
        public static double kP = 6.6544;
        public static double kI = 0;
        public static double kD = 4.833;

        /* Feedforward Constants */
        public static double kS = 0.402;
        public static double kCos = 0.771;
        public static double kV = 0.758;
        public static double kA = 0.00717;

    }

    public static class DrivetrainConstants {
        public static final int
        /* Drivetrain motor IDs */ 
            leftMaster = 3, // TalonFX right Masters & Slaves currently reversed
            leftSlave = 1, // TalonFX
            rightMaster = 4, // TalonFX
            rightSlave = 2; // TalonFX
        
        /* feedforward constants */
        public static final double kS = 0.66589; // voltage required to overcome friction (V)
        public static final double kV = 2.4372; // voltage over velocity (V/(meters/second))
        public static final double kA = 0.28968; // voltage over acceleration (V(meters/second/second))

        /* PID constants */
        public static final double kP = 3.2181;
        public static final double kI = 0;
        public static final double kD = 0;

        /* Wheels Constants */
        public static final double kTicksPerRotation = 2048 * 10.71; // Falcon 500 integrated encoder (2048 CPR)
                                                                     // multiplied by gear ratio (10.42:1)
        public static final double kWheelDiameter = Units.InchesToMeters(6);

        public static final double kMaxSpeedMPS = 3.726; // max speed in meters per second
        public static final double kMaxAcceleration = 0; //max acceleration in meters per second per second
        public static final double kTrackWidth = 0.7051868402911773; // distance between wheels
        public static final double kMaxTurnRate = -5.283706; //Max turn rate in radians per second
        public static final double kMaxCurvature = kMaxTurnRate / kMaxSpeedMPS; // Maximum turn rate in radians per meter TODO: update

        public static final double sdx = 0.2;

        public static final double kPV = 0;
    }

    public static class IntakeConstants {
        /* Motors */
        public static final int rollerMotor = 3;
    }


    public static class VisionConstants {
        /*
            Servo Constants
        */
        //public static final int servoChannel = 0;

        /* Color Sensor Constants */
        public static final int minimumProximity = 450;
        public static final double minimumSimilarity = 0.6;

        /* Turn PID Constants */
        public static double kPTurn = 0.1;
        public static double kITurn = 0;
        public static double kDTurn = 0.0035;
        public static double kTurnTolerance = 1.07;

        /* Distance PID Constants */
        public static double kPDist = 0.1;
        public static double kIDist = 0;
        public static double kDDist = 0;
        public static double kDistTolerance = 0;
        public static double kYDesired = 0.0; //For proper shooting distance
        /* For calculating distance from goal */
        public static double mountAngle = 48; //TODO: verify distance constants
        public static double goalHeightInches = 104;
        public static double limelightHeightInches = 25.5;
    }
}