package frc.robot.commands;

import java.util.Set;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Units.DrivetrainUnits;
import frc.robot.subsystems.Drivetrain;

public class TurnXDegrees implements Command {
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private Subsystem[] requirements = {drivetrain};
    private TrapezoidProfile.State goal;
    private TrapezoidProfile profile;
    private TrapezoidProfile.Constraints constraints;
    private PIDController turnController;
    private double startTime;

    private double angVel; 
    private double angAcc;
    private double degrees;
    public enum Gear { 
        forwards, reverse;
    }
    private Gear gear;
    private double startAngle;
    

    /*
        @param distance     desired distance
        @param maxSpeedMPS  max speed during motion
        @param maxAccelMPSS max acceleration during motion
    */
    public TurnXDegrees(double degrees, double angVel, double angAcc) {
        turnController = new PIDController(
            DrivetrainConstants.kP, 
            DrivetrainConstants.kI, 
            DrivetrainConstants.kD
        );
        degrees %= 360;
        turnController.setTolerance(5);
        constraints = new TrapezoidProfile.Constraints(angVel, angAcc);
        goal = new TrapezoidProfile.State(degrees, 0.0);
        profile = new TrapezoidProfile(constraints, goal);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        drivetrain.resetEncoders();
        //startAngle = RobotContainer.navX.getAngle();

    }

    @Override
    public void execute() {
        double timestamp = Timer.getFPGATimestamp() - startTime;
        TrapezoidProfile.State profileCalc = profile.calculate(timestamp);
        double left, right;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, profileCalc.velocity * Math.PI/180);
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
            DrivetrainConstants.kTrackWidth
        );
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        left = Drivetrain.FEEDFORWARD.calculate(wheelSpeeds.leftMetersPerSecond);
        right = Drivetrain.FEEDFORWARD.calculate(wheelSpeeds.rightMetersPerSecond);

        SmartDashboard.putNumber("leftwheel", left);
        SmartDashboard.putNumber("rightwheel", right);
        // double turn = turnController.calculate(RobotContainer.navX.getAngle() - startAngle, profileCalc.position);
        // left += turn;
        // right -= turn;
        // left += Drivetrain.LEFT_PID_CONTROLLER.calculate(
        //     RobotContainer.navX.getAngle(), 
        //     profileCalc.position
        // );
        // right += Drivetrain.RIGHT_PID_CONTROLLER.calculate(
        //     RobotContainer.navX.getAngle(), 
        //     profileCalc.position
        // );
        Drivetrain.setOpenLoop(left/ Constants.kMaxVoltage, right / Constants.kMaxVoltage);
        SmartDashboard.putString("TurnXFinishedalt", "No");
        //profile = new TrapezoidProfile(constraints, goal, profileCalc);
    }

    @Override 
    public boolean isFinished() {
        return profile.isFinished(Timer.getFPGATimestamp() - startTime);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("TurnXFinishedalt", "Yes");
        drivetrain.stop();
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}