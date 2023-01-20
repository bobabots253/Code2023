package frc.robot.subsystems;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Units;
import frc.robot.Util;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain implements Subsystem {
    private static Drivetrain instance = null;
    private static final TalonFX
        leftMaster = Util.createTalonFX(DrivetrainConstants.leftMaster),
        leftSlave = Util.createTalonFX(DrivetrainConstants.leftSlave),
        rightMaster = Util.createTalonFX(DrivetrainConstants.rightMaster),
        rightSlave = Util.createTalonFX(DrivetrainConstants.rightSlave);
    
    public static final List<TalonFX> motors = List.of(leftMaster, leftSlave, rightMaster , rightSlave);


    public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidth);
    public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV, DrivetrainConstants.kA);
    public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(DrivetrainConstants.kMaxSpeedMPS, DrivetrainConstants.kMaxAcceleration);
    public static final ProfiledPIDController LEFT_PID_CONTROLLER = new ProfiledPIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD, constraints);
    public static final ProfiledPIDController RIGHT_PID_CONTROLLER = new ProfiledPIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD, constraints);
    public static DifferentialDriveOdometry ODOMETRY = new DifferentialDriveOdometry(Rotation2d.fromDegrees(RobotContainer.navX.getAngle()), getLeftEnc(), getRightEnc());
    private Drivetrain() {
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        //leftSlave.setNeutralMode(NeutralMode.Coast);
        //rightSlave.setNeutralMode(NeutralMode.Coast);
        // Inverting opposite sides of the drivetrain
        List.of(leftMaster , leftSlave).forEach(motor -> motor.setInverted(false));
        List.of(rightMaster , rightSlave).forEach(motor -> motor.setInverted(true));

        register();
    }

    @Override
    public void periodic() {
        ODOMETRY.update(Rotation2d.fromDegrees(-RobotContainer.navX.getAngle()),
        getLeftEncMeters(),
        getRightEncMeters());
        SmartDashboard.putNumber("Left Master output: ", getLeftEncVelocityMeters());
        //SmartDashboard.putNumber("Left Slave output: ", leftSlave.getMotorOutputPercent());
        SmartDashboard.putNumber("Right Master output: ", getRightEncVelocityMeters());
        SmartDashboard.putNumber("NavX heading", RobotContainer.navX.getAngle());
        //SmartDashboard.putNumber("Right Slave output: ", rightSlave.getMotorOutputPercent());
        
    }
    private static int kInverted = 1; //1 or -1
    public static int getkInvert() { //only for teleop driving, up to user to read this flag
        return kInverted;
    }

    public static void setOpenLoop(double left, double right) {
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
    }

    public static void setVoltages(double leftv, double rightv) {
        setOpenLoop(leftv/Constants.kMaxVoltage, rightv/Constants.kMaxVoltage);
    }

    public static void setInverted(boolean status) { //For defense, the back of the robot becomes the front
        if(status) {
            kInverted = -1;
        } else {
            kInverted = 1;
        }
    }

    public void stop() {
        setOpenLoop(0, 0);
    }

    /**
     * Zeroes encoders
     */
    public void resetEncoders() {
        resetEncoders(0, 0);
    }
    
    /**
     * Sets encoders to a specific value
     * @param left  left wheel value
     * @param right right wheel value
     */
    public void resetEncoders(int left, int right) {
        rightMaster.setSelectedSensorPosition(right);
        leftMaster.setSelectedSensorPosition(left);
    }

    /**
     * @return the left and right drivetrain velocities (in meters/sec) as a DifferentialDriveWheelSpeeds object
     */
    public static DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncVelocityMeters(), getRightEncVelocityMeters());
    }
    
    /**
     * @return the current position measurement of the left drivetrain encoder in talon native units (ticks)
     */
    public static double getLeftEnc() {
        return leftMaster.getSelectedSensorPosition();
    }
    
    /**
     * @return the current position measurement of the right drivetrain encoder in talon native units (ticks/)
     */
    public static double getRightEnc() {
        return rightMaster.getSelectedSensorPosition();
    }

    /**
     * @return the current position measurement of the left drivetrain encoder in meters
     */
    public static double getLeftEncMeters() {
        return Units.DrivetrainUnits.TicksToMeters(getLeftEnc());
    }
    
    /**
     * @return the current position measurement of the right drivetrain encoder in meters
     */
    public static double getRightEncMeters() {
        return Units.DrivetrainUnits.TicksToMeters(getRightEnc());
    }


    
    /**
     * @return the current velocity measurement of the left drivetrain encoder in talon native units (ticks/ds)
     */
    public static double getLeftEncVelocity() {
        return leftMaster.getSelectedSensorVelocity();
    }
    
    /**
     * @return the current velocity measurement of the right drivetrain encoder in talon native units (ticks/ds)
     */
    public static double getRightEncVelocity() {
        return rightMaster.getSelectedSensorVelocity();
    }

    /**
     * @return the current velocity measurement of the left drivetrain encoder in meters
     */
    public static double getLeftEncVelocityMeters() {
        return Units.DrivetrainUnits.TicksPerDecisecondToMPS(getLeftEncVelocity());
    }

    /**
     * @return the current velocity measurement of the right drivetrain encoder in meters
     */
    public static double getRightEncVelocityMeters() {
        return Units.DrivetrainUnits.TicksPerDecisecondToMPS(getRightEncVelocity());
    }
    
    /* Static class to contain the speeds of each side of the drivetrain */
    public static class WheelState {
        public double left, right;
        
        public WheelState(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    public static Drivetrain getInstance() {
        if(instance == null) instance = new Drivetrain();
        return instance;
    }
}