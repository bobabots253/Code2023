package frc.robot.subsystems;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Util;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.WristConstants;

public class Intake implements Subsystem {

    private static final CANSparkMax intakeMotor = Util.createSparkMAX(WristConstants.intakeMotor, MotorType.kBrushless);
    private static int LBcount = 0, RBcount = 0;
    private static LinearFilter filter;
    private static Intake instance;
    public static boolean cone;
    public static boolean running;
    public static boolean isReleased;
    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }

    public Intake(){
        filter = LinearFilter.movingAverage(30);
        // cone = false;
        running = false;
        isReleased = false;
        register();
    }

    public enum ScorePos {
        HIGH(1), MID(2), LOW(3), STOW(4);
        public int val;
        private ScorePos(int val) {
            this.val = val;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("CONE?", cone);
        SmartDashboard.putNumber("INTAKE CURRENT", intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("INTAKE VOLTAGE", intakeMotor.getAppliedOutput());
        SmartDashboard.putBoolean("INTAKE running", running);

        if (RobotContainer.getOperatorLT() > 0 && RobotContainer.getOperatorRT() == 0) {
            SmartDashboard.putNumber("OPERATOR LT", RobotContainer.getOperatorLT());
            intakeMotor.setSmartCurrentLimit(30);
            set(-1.);
            isReleased = false;

        }
        
        if (RobotContainer.getOperatorRT() > 0 && RobotContainer.getOperatorLT() == 0) {
            intakeMotor.setSmartCurrentLimit(30);
            set(1);
            isReleased = false;
        }

        if (RobotContainer.getOperatorLT() == 0 && RobotContainer.getOperatorRT() == 0 && !isReleased) {
            set(0.);
            isReleased = true;
        }

        int highcurrlimit = 20;
        int lowcurrlimit = 3;

        SmartDashboard.putBoolean("OPERATOR VIEW", RobotContainer.operator_VIEW.getAsBoolean());

        // if (RobotContainer.driverController.getLeftBumperPressed() || RobotContainer.operatorController.getLeftBumperPressed()) {
        //     LBcount++;
        //     set(1);
        //     intakeMotor.setSmartCurrentLimit(highcurrlimit);
        //     cone = true;
        //     running = true;
        // }
        // if (LBcount == 2) {
        //     set(0.);
        //     LBcount = 0;
        //     running = false;
        // }
        double filterOutput = filter.calculate(intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("INTAKE filtered output", filterOutput);
        if (filterOutput > 25.0) {
            intakeMotor.setSmartCurrentLimit(lowcurrlimit);
            if (cone) set(.1);
            else set(-.1);
        }

        // if (RobotContainer.driverController.getRightBumperPressed() || RobotContainer.operatorController.getRightBumperPressed()) {
        //     RBcount++;
        //     set(-1);
        //     cone = false;
        //     intakeMotor.setSmartCurrentLimit(highcurrlimit);
        //     running = true;
        // }
        // if (RBcount == 2) {
        //     set(0.);
        //     RBcount = 0;
        //     running = false;
        // }

        
    }

    public void runOff() {
        Intake.running = false;
    }

    public void coneIntake() {
        Intake.cone = true;
    }

    public void cubeIntake() {
        Intake.cone = false;
    }
    public void setCurrLimit(int limit) {
        intakeMotor.setSmartCurrentLimit(limit);
    }
    public void set(double value) {
        intakeMotor.set(value);
        SmartDashboard.putNumber("INTAKE speed", value);
    }

    public void setAuto(double value) {
        if (cone) set(value);
        else set(-value);
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
        intakeMotor.set(value);
        int limit = 40;
        int lowLim = 20;
        if (current > limit) {
            intakeMotor.set(value/3.);
        } else {
            intakeMotor.set(value);
        }
       
    }
    /**
     * Stops the intake
     */
    public void stopIntake() {
        intakeMotor.set(0);
    }
}
