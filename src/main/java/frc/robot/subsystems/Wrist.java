package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Wrist extends ProfiledPIDSubsystem {
    
    private static final CANSparkMax intakeMotor = Util.createSparkMAX(WristConstants.intakeMotor, MotorType.kBrushless);
    //private static CANSparkMax conveyorMotor;
    private static final CANSparkMax wristMotor = Util.createSparkMAX(WristConstants.wristMotor, MotorType.kBrushless);
    private static final RelativeEncoder wristEncoder = wristMotor.getEncoder();
    private static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(ArmConstants.kS, ArmConstants.kCos, ArmConstants.kV, ArmConstants.kA);
    private SparkMaxPIDController pidController;
    // private RelativeEncoder relWristEncoder = wristMotor.getEncoder();

    private static Wrist instance;
    public static Wrist getInstance(){
        if (instance == null) instance = new Wrist();
        return instance;
    }

    private Wrist(){

        super(
            new ProfiledPIDController(
                WristConstants.kP, 
                WristConstants.kI, 
                WristConstants.kD,
                new TrapezoidProfile.Constraints(WristConstants.kMaxVelocity, WristConstants.kMaxAcceleration)
            ),
            0
        );
        intakeMotor.setInverted(true);
        wristMotor.setInverted(false);
        //wristEncoder.setPositionConversionFactor(2*Math.PI/WristConstants.gearRatio);
        wristEncoder.setPosition(0);
        pidController = wristMotor.getPIDController();
        pidController.setP(0.1);
        pidController.setI(0);
        pidController.setD(0);
        pidController.setIZone(0);
        pidController.setFF(0);
        pidController.setOutputRange(-0.3, 0.3);
        //conveyorMotor = Util.createSparkMAX(ConveyorConstants.motor, MotorType.kBrushless);
        // conveyorMotor.setInverted(true);
        // conveyorMotor.burnFlash();
        /*conveyorMotor.setInverted(false);
        conveyorMotor.burnFlash();*/
        register();
    }

    /**
     * Sets the conveyor to spin at a percent of max speed
     * @param value Percent speed
     */
    /*public void setConveyor(double value) {
        conveyorMotor.set(value);
    }*/

    /**
     * Sets the intake to spin at a given voltage
     * 
     * @param value Percent of maximum voltage to send to motor
     */
    public void intake(double value) {
        intakeMotor.set(value);
    }
    // add methods to spin in opposite direction
    public void setWrist(double value) {
        wristMotor.set(value);
    }
    
    /**
     * Stops the intake
     */
    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void stopWrist() {
        wristMotor.set(0);
    }

    public void periodic() {
        SmartDashboard.putNumber(
            "WRIST: Angle", wristEncoder.getPosition()*(180.0/Math.PI)
        );
        SmartDashboard.putNumber("WRIST encoder value", wristEncoder.getPosition());
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        // TODO Auto-generated method stub
        double feedforward = FEEDFORWARD.calculate(setpoint.position, setpoint.velocity);
        wristMotor.setVoltage(output + feedforward);
        
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return wristEncoder.getPosition();
    }
    
    // public boolean getIntakeSensor() {
    //     if(!Robot.useV3()) {
    //         return !intakePhotoelectric.get();
    //     } else {
    //         return false; //(RobotContainer.colorSensorV3.getProximity() >= ConveyorConstants.minimumProximity);
    //     }
    // }

    public void setWristPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("SetWristPoint", position);
    }
}
