package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ConveyorConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake implements Subsystem {
    
    private static final CANSparkMax rollerMotor = Util.createSparkMAX(IntakeConstants.rollerMotor, MotorType.kBrushless);
    //private static CANSparkMax conveyorMotor;
    private static final CANSparkMax conveyorMotor = Util.createSparkMAX(ConveyorConstants.conveyorMotor, MotorType.kBrushless);
    
    private static final DigitalInput intakePhotoelectric = new DigitalInput(ConveyorConstants.intakePhotoelectric); //sensor closest to intake
    private static Intake instance;
    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }

    private Intake(){
        conveyorMotor.setInverted(true);
        rollerMotor.setInverted(false);
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
        rollerMotor.set(value);
    }
    public void setConveyor(double value) {
        conveyorMotor.set(value);
    }
    
    /**
     * Stops the intake
     */
    public void stopIntake() {
        rollerMotor.set(0);
        conveyorMotor.set(0);
    }

    public void stopConveyor(){
        conveyorMotor.set(0);
    }
    
    public boolean getIntakeSensor() {
        if(!Robot.useV3()) {
            return !intakePhotoelectric.get();
        } else {
            return false; //(RobotContainer.colorSensorV3.getProximity() >= ConveyorConstants.minimumProximity);
        }
    }
}
