package frc.robot.commands;
//thank you william our lord and savior
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.CustomUtil.Timeframe;
import frc.robot.RobotContainer.IntakeVisionPipeline;
import frc.robot.RobotContainer.LEDMode;
import frc.robot.RobotContainer.ShooterVisionPipeline;
import frc.robot.subsystems.Drivetrain;

public class GridTrack implements Command {
    private static final PIDController TURN_PID_CONTROLLER = new PIDController(VisionConstants.kPTurn,
            VisionConstants.kITurn, VisionConstants.kDTurn);
    private static final PIDController DIST_PID_CONTROLLER = new PIDController(VisionConstants.kPDist,
            VisionConstants.kIDist, VisionConstants.kDDist);
    private Subsystem[] requirements = { Drivetrain.getInstance()};
    private Timeframe<Integer> timeframe;
    public GridTrack() {
        timeframe = new Timeframe<>(1.5, 1.0/Constants.dt);
    }

    @Override
    public void initialize() {
        timeframe.reset();
        RobotContainer.getInstance().setLEDMode(LEDMode.ON);
        RobotContainer.getInstance().setPipeline(IntakeVisionPipeline.ROBOT);//we will only have 1 pipeline
    }

    boolean atTarget;
    @Override
    public void execute() {
        double left, right;
        double turnError = RobotContainer.getXOffset();
        //double distError = VisionConstants.kYDesired - RobotContainer.getYOffset();
        SmartDashboard.putNumber("x off", RobotContainer.getXOffset());
        SmartDashboard.putNumber("y off", RobotContainer.getYOffset());

        //if (Math.abs(turnError) < VisionConstants.kTurnTolerance) turnError = 0;
        //if (distError < VisionConstants.kDistTolerance) distError = 0;
        //double throttle = DIST_PID_CONTROLLER.calculate(distError, 0);
        double turn = TURN_PID_CONTROLLER.calculate(turnError, 0);
        SmartDashboard.putNumber("turn", turn);
        
        // Turns in place when there is no throttle input
        left = turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;
        right = -turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;
        double maxMagnitude;
        /*if((maxMagnitude = Math.max(Math.abs(left), Math.abs(right))) > DriverConstants.kTurnInPlaceSens) {
            left = left / maxMagnitude * DriverConstants.kTurnInPlaceSens;
            right = right / maxMagnitude * DriverConstants.kTurnInPlaceSens;
        }*/
        left = Drivetrain.FEEDFORWARD.calculate(left) / Constants.kMaxVoltage;
        right = Drivetrain.FEEDFORWARD.calculate(right) / Constants.kMaxVoltage;
        atTarget = (int) Math.abs(turnError) <= VisionConstants.kTurnTolerance;
        if(atTarget) { //TODO: Test timeframe and if it works well, tune the desired "matching percentage"
            timeframe.update(1);
        } else {
            timeframe.update(0);
        }
        SmartDashboard.putNumber("AtTarget?", (atTarget) ? 1 : 0);
        SmartDashboard.putNumber("Mean Accuracy", (timeframe.getAverageValue()));
        Drivetrain.setOpenLoop(left, right);

    }

    @Override
    public boolean isFinished() {
         return timeframe.percentEqual(1) >= 0.85;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.getInstance().setLEDMode(LEDMode.ON);
        RobotContainer.getInstance().setPipeline(IntakeVisionPipeline.DRIVER); //change
        Drivetrain.setOpenLoop(0.0, 0.0);
        timeframe.reset();
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
