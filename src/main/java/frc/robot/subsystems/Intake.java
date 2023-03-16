package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.Util;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.WristConstants;

public class Intake implements Subsystem {

    private static final CANSparkMax intakeMotor = Util.createSparkMAX(WristConstants.intakeMotor, MotorType.kBrushless);

    private static Intake instance;
    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }

    public Intake(){
        register();
    }

    @Override
    public void periodic() {
        /*
        What should happen periodically?
         */

        double LTvalue = RobotContainer.getOperatorLT();
        double RTvalue = RobotContainer.getOperatorRT();
        if(LTvalue > 0. && RTvalue == 0.){
            set(-0.75);
        }else if(LTvalue == 0 && RTvalue ==0){
            set(0.);
        }
        

        if(RTvalue > 0. && LTvalue == 0.){
            set(0.75);
        }else if (LTvalue==0 && RTvalue == 0){
            set(0.);
        }
        SmartDashboard.putNumber("RTval", RTvalue);
        
    }

    public void set(double value) {
        intakeMotor.set(value);
    }
    //hold methods periodically move the motor to avoid stalling (burnout) press button only once pls :>
    public void holdCone(double value) {
        Timer t = new Timer();
        t.start();
       
        if (t.get() == 2) {
            t.restart();
            intakeMotor.set(value);
            if (t.hasElapsed(1)) {
             stopIntake();
            }
        }
    }
    public void holdCube(double value) { // put in pos value
        value *=-1;
        Timer t = new Timer();
        t.start();
       
        if (t.get() % 2 == 0) {
            t.restart();
            intakeMotor.set(value);
            if (t.hasElapsed(1)) {
             stopIntake();
            }
        }
    }
    public void setHold(double value) {
        double current = intakeMotor.getOutputCurrent();
        intakeMotor.setSmartCurrentLimit(40);
        intakeMotor.set(value);
       
    }
    /**
     * Stops the intake
     */
    public void stopIntake() {
        intakeMotor.set(0);
    }
}
