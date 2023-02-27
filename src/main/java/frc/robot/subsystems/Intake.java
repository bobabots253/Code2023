package frc.robot.subsystems;

import frc.robot.Util;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.WristConstants;

public class Intake implements Subsystem {

    private static final CANSparkMax intakeMotor = Util.createSparkMAX(WristConstants.intakeMotor, MotorType.kBrushless);

    private static Intake instance;
    public static Intake getInstance(){
        /*
         * return an instance of intake, but make sure to return the same instance every time!
         */
        return null;
    }
    @Override
    public void periodic() {
        /*
        What should happen periodically?
         */
    }

    public void set(double value) {
        /*
         * Figure out how to set the intake motor given a percentage
         */
    }

    /**
     * Stops the intake
     */
    public void stopIntake() {
        /*
         * figure out how to stop the intake motor
         */
    }
}
