package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Util;                                     
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;


public class Arm extends ProfiledPIDSubsystem {
    private static final CANSparkMax motor = Util.createSparkMAX(ArmConstants.actuateMotor, MotorType.kBrushless);
    
    private SparkMaxAbsoluteEncoder armEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    private RelativeEncoder relArmEncoder = motor.getEncoder();
    //private static final Encoder armEncoder = new Encoder(4,3);
    
    private static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(ArmConstants.kS, ArmConstants.kCos, ArmConstants.kV, ArmConstants.kA);
    
    private static Arm instance;
    public static Arm getInstance() {
        if(instance == null) instance = new Arm();
        return instance;
    }
    
    /**
     * Enum class representing the two possible positions of the intake arm, UP and DOWN
     */
    // public enum State {
    //     STORED(0), OUT(ArmConstants.kArmOffset); //Stored is the starting position, where the arm is stored in the bot, while out is used for intake
        
    //     public double position;
        
    //     /**
    //      * @param position the value of the arm position in radians
    //      */
    //     private State(double position) {
    //         this.position = position;
    //     }
    // }
    
    private Arm() {
        super(
            new ProfiledPIDController(
                ArmConstants.kP, 
                ArmConstants.kI, 
                ArmConstants.kD,
                new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocity, ArmConstants.kMaxAcceleration)
            ),
            0
        );
        motor.setSmartCurrentLimit(2); 
        /*
        motor.configContinuousCurrentLimit(1);
        motor.configPeakCurrentLimit(0);
        motor.enableCurrentLimit(true);
        */
        armEncoder.setPositionConversionFactor(2.0 * Math.PI);
        motor.setInverted(false);
        motor.burnFlash();
        setGoal(ArmConstants.kStartRads);
        // disable();
        register();
    }

    // public void setGoal(State goal) {
    //     setGoal(goal.position);
    // }

    public void setOpenLoop(double value) {
        SmartDashboard.putNumber("Commanded arm actuation", value);
        motor.set(value);
    }
    /**
     * Set the intake to rotate manually (overriding the position control)
     * @param value Percent of maximum voltage to send to motor
     */
    public void rotate(double value) {
        motor.set(value);
    }
    
    public void stopArm() {
        disable();
        motor.set(0);//makes it so it moves the same as the spring pulling, comes from testing :(
    }
    
    /**
     * Resets encoders to zero
     */
    public void resetEncoders() {
        relArmEncoder.setPosition(0.0);
    }
    
    @Override
    public void periodic() {
        //super.periodic();

        SmartDashboard.putNumber("encoder value", armEncoder.getPosition() * 2 * Math.PI);
        SmartDashboard.putNumber("measurement", getMeasurement());
    }
    
    /**
     * @return the arm's current position as a radian measure
     */
    @Override
    public double getMeasurement() {
        return armEncoder.getPosition();
    }
    
    /**
     * Uses the value calculated by ProfiledPIDSubsystem
     */
    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate feedforward from the setpoint
        //FEEDFORWARD.calculate(setpoint.position, setpoint.velocity);
        // Set motor, converting voltage to percent voltage

        double feedforward = FEEDFORWARD.calculate(setpoint.position, setpoint.velocity);
        motor.setVoltage(output + feedforward);

        // motor.set(setpoint.velocity/13.209 + output/12); //without feedforward, use PID to correct error

        // SmartDashboard.putNumber("pos", setpoint.position);
        // SmartDashboard.putNumber("output", output/12);
        // //SmartDashboard.putNumber("feedforward + output", (output+feedforward)/12);

    }
}
