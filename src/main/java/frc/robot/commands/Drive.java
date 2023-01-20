package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.Set;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class Drive implements Command {
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private Subsystem[] requirements = {drivetrain};


    public Drive(State state) {
        this.state = state;
    }

    public enum State {
        CurvatureDrive2019, CheesyDriveOpenLoop, CheesyDriveClosedLoop, TankDrive
    }
    
    private State state;

    @Override
    public void execute(){
        // Retrieving the deadbanded throttle and turn values (the controller joystick values)
        double throttle = RobotContainer.getThrottle();
        double turn = RobotContainer.getTurn(); //* Drivetrain.getkInvert();
        if(throttle != 0) { //TODO: test reverse driving in offense and defense modes
            turn *= /*Math.signum(throttle) * */ Drivetrain.getkInvert();
            throttle *= Drivetrain.getkInvert();
        }
        
        double altThrottle = RobotContainer.getAltThrottle();
        SmartDashboard.putNumber("turn input", turn);
        SmartDashboard.putNumber("throttle input", throttle);

        double left, right;

        switch(state) {
            case CurvatureDrive2019:
                // Differential drive as long as throttle is greater than zero (deadbanded).
                if (throttle != 0) {
                    left = (throttle + throttle * turn * DriverConstants.kTurnSens) * DriverConstants.kDriveSens;
                    right = (throttle - throttle * turn * DriverConstants.kTurnSens) * DriverConstants.kDriveSens;

                    // Normalize
                    double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
                    
                    if(maxMagnitude > DriverConstants.kDriveSens) {
                        left = left / maxMagnitude * DriverConstants.kDriveSens;
                        right = right / maxMagnitude * DriverConstants.kDriveSens;
                    } 
                } else {
                    // Turns in place when there is no throttle input
                    left = turn * DriverConstants.kTurnInPlaceSens;
                    right = -turn * DriverConstants.kTurnInPlaceSens;
                }

                break;
            case CheesyDriveOpenLoop:
                if (throttle != 0) {
                    throttle *= DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kDriveSens;
                    turn *= DrivetrainConstants.kMaxCurvature * DriverConstants.kTurnSens * throttle; 

                    DifferentialDriveWheelSpeeds wSpeeds = Drivetrain.KINEMATICS.toWheelSpeeds(new ChassisSpeeds(throttle, 0, turn));
                    wSpeeds.desaturate(DrivetrainConstants.kMaxSpeedMPS);

                    left = Drivetrain.FEEDFORWARD.calculate(wSpeeds.leftMetersPerSecond) / Constants.kMaxVoltage;
                    right = Drivetrain.FEEDFORWARD.calculate(wSpeeds.rightMetersPerSecond) / Constants.kMaxVoltage;

                } else {
                    // Turns in place when there is no throttle input
                    left = turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;
                    right = -turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;

                    left = Drivetrain.FEEDFORWARD.calculate(left) / Constants.kMaxVoltage;
                    right = Drivetrain.FEEDFORWARD.calculate(right) / Constants.kMaxVoltage;
                }

                break;

            case CheesyDriveClosedLoop:
                if (throttle != 0) {
                    throttle *= DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kDriveSens;
                    turn *= DrivetrainConstants.kMaxCurvature * DriverConstants.kTurnSens * throttle;

                    DifferentialDriveWheelSpeeds _wSpeeds = Drivetrain.KINEMATICS.toWheelSpeeds(new ChassisSpeeds(throttle, 0, turn));
                    _wSpeeds.desaturate(DrivetrainConstants.kMaxSpeedMPS);

                    left = Drivetrain.FEEDFORWARD.calculate(_wSpeeds.leftMetersPerSecond);
                    right = Drivetrain.FEEDFORWARD.calculate(_wSpeeds.rightMetersPerSecond);

                    left += Drivetrain.LEFT_PID_CONTROLLER.calculate(Drivetrain.getLeftEncVelocityMeters(), _wSpeeds.leftMetersPerSecond);
                    right += Drivetrain.RIGHT_PID_CONTROLLER.calculate(Drivetrain.getRightEncVelocityMeters(), _wSpeeds.rightMetersPerSecond);
                    
                    // Convert voltages to percent voltages
                    left /= Constants.kMaxVoltage;
                    right /= Constants.kMaxVoltage;
                } else {
                    // Turns in place when there is no throttle input
                    left = turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;
                    right = -turn * DrivetrainConstants.kMaxSpeedMPS* DriverConstants.kTurnInPlaceSens;

                    left = Drivetrain.FEEDFORWARD.calculate(left);
                    right = Drivetrain.FEEDFORWARD.calculate(right);

                    left += Drivetrain.LEFT_PID_CONTROLLER.calculate(Drivetrain.getLeftEncVelocityMeters(), left);
                    right += Drivetrain.RIGHT_PID_CONTROLLER.calculate(Drivetrain.getRightEncVelocityMeters(), right);
                }
                
                break;
            case TankDrive:
                left = throttle * DriverConstants.kDriveSens;
                right = altThrottle * DriverConstants.kDriveSens;
                break;
            default:
                left = right = 0;
                break;
        }
        SmartDashboard.putNumber("left dt commanded", left);
        SmartDashboard.putNumber("right dt commanded", right);
        drivetrain.setOpenLoop(left, right); 
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setOpenLoop(0, 0);
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
