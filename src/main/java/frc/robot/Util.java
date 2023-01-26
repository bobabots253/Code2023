package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;

public class Util {

    private static int talonSRXDefaultContinuousLimit = 38;
    private static int talonSRXDefaultPeakLimit = 45;
    private static int talonSRXDefaultPeakDuration = 125;

    public static final boolean talonFXStatorLimitEnable = false;
    public static final double talonFXStatorCurrentLimit = 100;
    public static final double talonFXStatorTriggerThreshold = 100;
    public static final double talonFXStatorTriggerDuration = 0;

    public static final boolean talonFXSupplyLimitEnable = false;
    public static final double talonFXSupplyCurrentLimit = 70;
    public static final double talonFXSupplyTriggerThreshold = 70;
    public static final double talonFXSupplyTriggerDuration = 0.7;

    private static int sparkMAXDefaultCurrentLimit = 40;

    private static double voltageCompensation = Constants.kMaxVoltage;
    

    /**
     * Create a CANSparkMax with current limiting enabled
     * 
     * @param id the ID of the Spark MAX
     * @param motortype the type of motor the Spark MAX is connected to 
     * @param stallLimit the current limit to set at stall
     * 
     * @return a fully configured CANSparkMAX
     */
    public static CANSparkMax createSparkMAX(int id, MotorType motortype, int stallLimit) {
        CANSparkMax sparkMAX = new CANSparkMax(id, motortype);
        /*sparkMAX.restoreFactoryDefaults();
        sparkMAX.enableVoltageCompensation(voltageCompensation);
        sparkMAX.setSmartCurrentLimit(stallLimit);
        sparkMAX.setIdleMode(IdleMode.kCoast);*/

        //sparkMAX.burnFlash();
        return sparkMAX;
    }

    /**
     * Create a CANSparkMax with default current limiting enabled
     * 
     * @param id the ID of the Spark MAX
     * @param motortype the type of motor the Spark MAX is connected to
     * 
     * @return a fully configured CANSparkMAX
     */
    public static CANSparkMax createSparkMAX(int id, MotorType motortype) {
        return createSparkMAX(id, motortype, sparkMAXDefaultCurrentLimit);
    }

    /**
     * Create a configured TalonFX 
     * 
     * @param id the ID of the motor
     * 
     * @return a fully configured TalonFX
     */
    public static TalonFX createTalonFX(int id) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.statorCurrLimit = new StatorCurrentLimitConfiguration(
            talonFXStatorLimitEnable, talonFXStatorCurrentLimit, talonFXStatorTriggerThreshold, talonFXStatorTriggerDuration);
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            talonFXSupplyLimitEnable, talonFXSupplyCurrentLimit, talonFXSupplyTriggerThreshold, talonFXSupplyTriggerDuration);
        config.voltageCompSaturation = 12;
        
        TalonFX talon = new TalonFX(id);
        talon.configFactoryDefault();
        talon.configAllSettings(config);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        return talon;

    }

    /*
    * Create a configured ColorSensorV3
    * @param id the ID of the motor
    * @return a fully configured ColorSensorV3
    */
    public static ColorSensorV3 createColorSensorV3(Port port) {
        ColorSensorV3 colorSensorV3 = new ColorSensorV3(port);
        return colorSensorV3;
    }
}