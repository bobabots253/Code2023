package frc.robot.commands;
import java.util.Set;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Units.DrivetrainUnits;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class DriveXMeters implements Command {
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private Subsystem[] requirements = {drivetrain};
    private TrapezoidProfile.State goal;
    private TrapezoidProfile profile;
    private TrapezoidProfile.Constraints constraints;
    private double startTime;
    public enum Gear { 
        forwards, reverse;
    }
    private Gear gear;
    

    /*
        @param distance     desired distance
        @param maxSpeedMPS  max speed during motion
        @param maxAccelMPSS max acceleration during motion
    */
    public DriveXMeters(double distance, double maxSpeedMPS, double maxAccelMPSS, Gear gear) {
        this.gear = gear;
        constraints = new TrapezoidProfile.Constraints(maxSpeedMPS, maxAccelMPSS);
        goal = new TrapezoidProfile.State(distance, 0.0);
        profile = new TrapezoidProfile(constraints, goal);
    }
    public DriveXMeters(double distance, double maxSpeedMPS, double maxAccelMPSS) {
        this(distance, maxSpeedMPS, maxAccelMPSS, Gear.forwards);
    }



    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        //TrapezoidProfile.State profileCalc = profile.calculate(Constants.dt);
        double timestamp = Timer.getFPGATimestamp() - startTime;
        TrapezoidProfile.State profileCalc = profile.calculate(timestamp);
        double left, right;
        left = Drivetrain.FEEDFORWARD.calculate(profileCalc.velocity);
        right = Drivetrain.FEEDFORWARD.calculate(profileCalc.velocity);
        left += Drivetrain.LEFT_PID_CONTROLLER.calculate(DrivetrainUnits.TicksToMeters(Drivetrain.getLeftEnc()) , (gear == Gear.reverse) ? -profileCalc.position : profileCalc.position);
        right += Drivetrain.RIGHT_PID_CONTROLLER.calculate(DrivetrainUnits.TicksToMeters(Drivetrain.getRightEnc()), (gear == Gear.reverse) ? -profileCalc.position : profileCalc.position);
        left /= Constants.kMaxVoltage;
        right /= Constants.kMaxVoltage;
        if(gear == Gear.reverse) {
            left *= -1;
            right *= -1;
        }
        drivetrain.setOpenLoop(left, right);
        //profile = new TrapezoidProfile(constraints, goal, profileCalc);
    }

    @Override 
    public boolean isFinished() {
        return profile.isFinished(Timer.getFPGATimestamp() - startTime);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}