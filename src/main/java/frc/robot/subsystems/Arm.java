package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer;
import frc.robot.Util;                                     
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;


public class Arm extends ProfiledPIDSubsystem {
    private static final CANSparkMax motorR = Util.createSparkMAX(3, MotorType.kBrushless);
    private static final CANSparkMax motorL = Util.createSparkMAX(1, MotorType.kBrushless);
    //private SparkMaxAbsoluteEncoder armEncoder = motorR.getAbsoluteEncoder(Type.kDutyCycle);
    private RelativeEncoder relArmEncoder = motorR.getEncoder();
    //private static final Encoder armEncoder = new Encoder(4,3);

    private SparkMaxPIDController pidController;
    
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
       
        /*
        motor.configContinuousCurrentLimit(1);
        motor.configPeakCurrentLimit(0);
        motor.enableCurrentLimit(true);
        */
        resetEncoders();
        // motorR.setSmartCurrentLimit(40); 
        // motorL.setSmartCurrentLimit(40);
        // motorL.follow(motorR);
        //armEncoder.setPositionConversionFactor(2.0 * Math.PI / WristConstants.gearRatio);
        // relArmEncoder.setPositionConversionFactor(360.0*ArmConstants.gearRatio);
        motorR.setInverted(false);
        motorL.setInverted(false);
        motorL.follow(motorR, true);
        // motor.burnFlash(); //dont burn flash through code - charles             *READ ME*
        setGoal(ArmConstants.kStartRads);
        // disable();
        motorR.setIdleMode(IdleMode.kBrake);
        motorL.setIdleMode(IdleMode.kBrake);

        pidController = motorR.getPIDController();
        pidController.setP(ArmConstants.kP); //0.1
        pidController.setI(ArmConstants.kI);//0.01
        pidController.setD(ArmConstants.kD);
        pidController.setIZone(0);
        pidController.setFF(0);
        pidController.setOutputRange(-0.3, 0.3);
        register();
    }

    // public void setGoal(State goal) {
    //     setGoal(goal.position);
    // }

    public void setOpenLoop(double value) {
        SmartDashboard.putNumber("Arm Commanded arm actuation", value);
        motorR.set(value);
        
    }
    
    public void stopArm() {
        // disable(); disable messes up the motor
        // motor.set(0);//makes it so it moves the same as the spring pulling, comes from testing :(
        setOpenLoop(0);
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
        //SmartDashboard.putNumber("absolut encoder", armEncoder.getPosition());
        //SmartDashboard.putNumber("encoder value", relArmEncoder.getPosition()*360.0/100.0); //johnny and charles r dumb
        SmartDashboard.putNumber("Arm encoder value", relArmEncoder.getPosition());
        SmartDashboard.putNumber("Arm measurement", getMeasurement());
        if (RobotContainer.getOperatorRightY() > 0.0) setOpenLoop(0.1);
        else if (RobotContainer.getOperatorRightY() < 0.0) setOpenLoop(-0.1);
        else stopArm();
        
    }
    
    /**
     * @return the arm's current position as a radian measure
     */
    @Override
    public double getMeasurement() {
        //return armEncoder.getPosition();
        return 0;
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
        motorL.setVoltage(output + feedforward);
        

        // motor.set(setpoint.velocity/13.209 + output/12); //without feedforward, use PID to correct error

        // SmartDashboard.putNumber("pos", setpoint.position);
        // SmartDashboard.putNumber("output", output/12);
        // //SmartDashboard.putNumber("feedforward + output", (output+feedforward)/12);

    }


    public void setArmPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("Arm SetPoint", position);
    }

    public void setArmPositionDegree(double degreePosition) {
        
    }
}
