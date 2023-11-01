package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.time.Instant;
import java.util.*;
import java.util.function.BiConsumer;

import frc.robot.Autonomous.Auto;
import frc.robot.Constants.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

// import org.photonvision.PhotonCamera; ???? maybe
// import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.GridTrack;

public class RobotContainer {
    private static RobotContainer instance = null;
    public static final XboxController driverController = new XboxController(Constants.InputPorts.driverController);
    public static final XboxController operatorController = new XboxController(Constants.InputPorts.operatorController);
    public static final Trigger driver_A = new JoystickButton(driverController, 1),
        driver_B = new JoystickButton(driverController, 2), driver_X = new JoystickButton(driverController, 3),
        driver_Y = new JoystickButton(driverController, 4), driver_LB = new JoystickButton(driverController, 5),
        driver_RB = new JoystickButton(driverController, 6), driver_VIEW = new JoystickButton(driverController, 7),
        driver_MENU = new JoystickButton(driverController, 8);
    public static final Trigger operator_A = new JoystickButton(operatorController, 1),
        operator_B = new JoystickButton(operatorController, 2), operator_X = new JoystickButton(operatorController, 3),
        operator_Y = new JoystickButton(operatorController, 4), operator_LB = new JoystickButton(operatorController, 5),
        operator_RB = new JoystickButton(operatorController, 6), operator_VIEW = new JoystickButton(operatorController, 7),
        operator_MENU = new JoystickButton(operatorController, 8);
    private static final POVButton operator_DPAD_UP = new POVButton(operatorController, 0),
    operator_DPAD_RIGHT = new POVButton(operatorController, 90), operator_DPAD_DOWN = new POVButton(operatorController, 180),
    operator_DPAD_LEFT = new POVButton(operatorController, 270);
    public static NetworkTable limelight;
    // private static final Spark ledStrip = new Spark(0);

    public static Drivetrain drivetrain;
    public static Intake intake;
    public static Wrist wrist;
    public static Arm arm;
    public static ColorSensorV3 colorSensorV3;
    public static AHRS navX;
    private RobotContainer() {
        navX = new AHRS(Port.kMXP);
        navX.calibrate();
        drivetrain = Drivetrain.getInstance();
        drivetrain.setDefaultCommand(new Drive(Drive.State.CurvatureDrive2019));

        //drivetrain.setDefaultCommand(new Drive(Drive.State.TankDrive));
        arm = Arm.getInstance();
        // intake = Intake.getInstance();
        wrist = Wrist.getInstance();
        intake = Intake.getInstance();
        limelight = NetworkTableInstance.getDefault().getTable("limelight"); //need to change
        setLEDMode(LEDMode.ON);
        
        drivetrain.resetEncoders();

        bindOI();
        initTrajectories();

        // ledStrip.set(0.81); //color: Aqua 
    }
    
    

    private void bindOI() {


        //driver_LB.whileTrue(new RunCommand(() -> wrist.setWrist(0.05), wrist));
        // driver_RB
        //     .whileTrue(new RepeatCommand(new RunCommand(() -> arm.setOpenLoop(0.1), arm)))
        //     .onFalse(new RunCommand(() -> arm.stopArm(), arm));
        // driver_LB
        //     .whileTrue(new RepeatCommand(new RunCommand(() -> arm.setOpenLoop(-0.1), arm)))
        //     .onFalse(new RunCommand(() -> arm.stopArm(), arm));
        // driver_A
        //     .whileTrue(new RepeatCommand(new RunCommand(() -> wrist.setWrist(0.1), wrist)))
        //     .onFalse(new RunCommand(() -> wrist.stopWrist(), wrist));
        driver_A
            .whileTrue(new RunCommand(() -> new GridTrack())); //might not stop, need to test
        driver_B
            //.whileTrue(new RepeatCommand(new RunCommand(() -> wrist.intake(0.4), wrist)))
            //.onFalse(new RunCommand(() -> wrist.stopIntake(), wrist));
            .onTrue(new RunCommand(() -> wrist.setWristPosition(0), wrist));
        // driver_Y
        //     //.whileTrue(new RepeatCommand(new RunCommand(() -> wrist.setWrist(-0.1), wrist)))
        //     //.onFalse(new RunCommand(() -> wrist.stopWrist(), wrist));
        //     .onTrue(new RunCommand(() -> arm.setArmPosition(-16), arm));
        driver_X
            //.whileTrue(new RepeatCommand(new RunCommand(() -> wrist.intake(-0.4), wrist)))
            //.onFalse(new RunCommand(() -> wrist.stopIntake(), wrist));
            .onTrue(new RunCommand(() -> wrist.setWristPosition(2.8), wrist));

        // Cube Intake Positions

        // operator_Y
        //     .onTrue(new RunCommand(() -> arm.setArmPosition(ArmConstants.kCubeHighScorePosition), arm))
        //     .onTrue(new RunCommand(() -> wrist.setWristPosition(WristConstants.kCubeHighScorePosition), wrist))
        //     .onFalse(new RunCommand(() -> arm.setArmPosition(ArmConstants.kStow), arm))
        //     .onFalse(new RunCommand(() -> wrist.setWristPosition(WristConstants.kStow), wrist));

        operator_Y
            .onTrue(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.HIGH), arm))
            .onTrue(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.HIGH), wrist))
            .onFalse(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.STOW), arm))
            .onFalse(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.STOW), wrist));

        // operator_A
        //     .onTrue(new RunCommand(() -> arm.setArmPosition(ArmConstants.kCubeFloorIntakePosition), arm))
        //     .onTrue(new RunCommand(() -> wrist.setWristPosition(WristConstants.kCubeFloorIntakePosition), wrist))
        //     .onFalse(new RunCommand(() -> arm.setArmPosition(ArmConstants.kStow), arm))
        //     .onFalse(new RunCommand(() -> wrist.setWristPosition(WristConstants.kStow), wrist));

        // operator_A
        //     // .onTrue(setArmWrist(Intake.ScorePos.LOW))
        //     // .onFalse(stow());
        //     .whileTrue(new SequentialCommandGroup(
        //         new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.LOW), arm).withTimeout(0.15),
        //         new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.LOW), wrist)
        //     ))
        //     // .onTrue(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.LOW), arm))
        //     // .onTrue(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.LOW), wrist))
        //     .onFalse(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.STOW), arm))
        //     .onFalse(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.STOW), wrist));

        operator_LB.whileTrue(setConeIntake());
        operator_RB.onTrue(setCubeIntake());

        operator_MENU
            // .onTrue(setArmWrist(Intake.ScorePos.LOW))
            // .onFalse(stow());
            .onTrue(intakeCommandShelf())
            // .onTrue(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.LOW), arm))
            // .onTrue(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.LOW), wrist))
            .onFalse(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.STOW), arm))
            .onFalse(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.STOW), wrist))
            //.onFalse(new InstantCommand(() -> intake.stopIntake(), intake))
            .onFalse(runningOff());

        operator_A
            // .onTrue(setArmWrist(Intake.ScorePos.LOW))
            // .onFalse(stow());
            .onTrue(intakeCommandb())
            // .onTrue(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.LOW), arm))
            // .onTrue(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.LOW), wrist))
            .onFalse(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.STOW), arm))
            .onFalse(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.STOW), wrist))
            //.onFalse(new InstantCommand(() -> intake.stopIntake(), intake))
            .onFalse(runningOff());


        operator_X
        .onTrue(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.MID), arm))
        .onTrue(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.MID), wrist))
        .onFalse(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.STOW), arm))
        .onFalse(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.STOW), wrist));

        operator_B
            .onTrue(new RunCommand(() -> arm.setArmPosition(ArmConstants.kStow), arm))
            .onTrue(new RunCommand(() -> wrist.setWristPosition(WristConstants.kStow), wrist));
            //Stow old finicky

        

        operator_VIEW
            .onTrue(new RunCommand(() -> arm.resetArmStow(), arm));
        // operator_B.onTrue(stow());

        // manual wrist and arm positioning
        operator_DPAD_UP.whileTrue(
            new RunCommand(() -> wrist.setWrist(0.25), wrist)
        ).onFalse(
            new RunCommand(() -> wrist.stopWrist(), wrist)
        );
        //     new RunCommand(() -> wrist.setWrist(0.5), wrist)

        operator_DPAD_DOWN.whileTrue(
            new RunCommand(() -> wrist.setWrist(-0.25), wrist)
        ).onFalse(
            new RunCommand(() -> wrist.stopWrist(), wrist)
        );
         // wristMotor.set(value); ???

        operator_DPAD_RIGHT.whileTrue(
            new RunCommand(() -> arm.setOpenLoop(0.1), arm)
        ).onFalse(
            new RunCommand(() -> arm.stopArm(), arm)
        );

        operator_DPAD_LEFT.whileTrue(
            new RunCommand(() -> arm.setOpenLoop(-0.1), arm)
        ).onFalse(
            new RunCommand(() -> arm.stopArm(), arm)
        );

    }

    public static Command setConeIntake() {
        return new InstantCommand(() -> intake.coneIntake(), intake);
    }

    public static Command setCubeIntake() {
        return new InstantCommand(() -> intake.cubeIntake(), intake);
    }
    public static Command runningOff() {
        return new InstantCommand(() -> intake.runOff(), intake);
    }
    public static Command intakeCommandShelf() {
        intake.setCurrLimit(30);
        Intake.running = true;
        Intake.isReleased = true;
        return new SequentialCommandGroup(
                new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.SHELF), arm).withTimeout(0.15),
                new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.SHELF), wrist)
            ).alongWith(new RunCommand(() -> intake.setAuto(1.), intake));
    }

    public static Command intakeCommand() {
        // double sspeed = 0.;
        // if (Intake.cone) sspeed = 1.;
        // else sspeed = -1.;
        //double sspeed = (Intake.cone) ? 1. : -1.;
        
        intake.setCurrLimit(30);
        Intake.running = true;
        Intake.isReleased = true;
        //intake.set(speed);
        return new SequentialCommandGroup(
                new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.LOW), arm).withTimeout(0.15),
                new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.LOW), wrist)
            ).alongWith(new RunCommand(() -> intake.setAuto(1.), intake));
    }

    public static Command intakeCommandb() {
        intake.setCurrLimit(30);
        Intake.running = true;
        Intake.isReleased = true;
        //intake.set(speed);
        return new SequentialCommandGroup(
                new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.LOW), arm).withTimeout(0.08),
                new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.LOW), arm).withTimeout(0.15),
                new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.LOW), wrist)
            ).alongWith(new RunCommand(() -> intake.setAuto(1.), intake));
    }

    public static Command intakeCommandParallel() {
        intake.setCurrLimit(30);
        Intake.running = true;
        Intake.isReleased = true;
        //intake.set(speed);
        return new ParallelCommandGroup(
                new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.LOW), arm),
                new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.LOW), wrist),
                new RunCommand(() -> intake.setAuto(1.), intake)
            );
    }
    public static Command setArmWrist(Intake.ScorePos pos) {
        return new RunCommand(() -> arm.setArmPositionAuto(pos), arm).alongWith(
               new RunCommand(() -> wrist.setWristPositionAuto(pos), wrist)
            );
    }

    public static Command stow() {
        return new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.STOW), arm).alongWith(
               new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.STOW), wrist)
            );
    }
    public static Command getAutonomousCommand(Auto.Selection selectedAuto) { //TODO: change auto based on selected strategy
        Command auto = null;
        switch (selectedAuto) {
            case MOVEARM:
                auto = Commands.runOnce(
                    () -> {
                        arm.setGoal(ArmConstants.autoDisplacementRads);
                        arm.enable();
                    }, 
                    arm
                );
                break;
            case MOVEWRIST:
                auto = Commands.runOnce(
                    () -> {
                        wrist.setGoal(WristConstants.autoDisplacementRads);
                        wrist.enable();
                    }, 
                    wrist
                );
                break;
            default:
                break;
        }
        return auto;
    }

    public static RobotContainer getInstance() {
        if(instance == null) instance = new RobotContainer();
        return instance;
    }

    
    public static Trajectory smallTraj = new Trajectory();
    private static final String smallJSON = "output/autoStep1.wpilib.json";
    private static final String[] auto1JSON = {"output/placeholder.wpilib.json"};
    public static Trajectory[] autoGroup1 = new Trajectory[1];

    private void initTrajectories() {
        smallTraj = initializeTrajectory(smallJSON);
        for(int i = 0; i < autoGroup1.length; i++) {
            autoGroup1[i] = initializeTrajectory(auto1JSON[i]);
        }
    }

    public static Trajectory initializeTrajectory(final String tjson) {
        Trajectory t = null;
        Path tPath = Filesystem.getDeployDirectory().toPath().resolve(tjson);
        try {
          t = TrajectoryUtil.fromPathweaverJson(tPath);
        } catch (IOException e) {
          System.out.println("PATHWEAVER NOT WORKING !!!!!! HEHEHHA");
          e.printStackTrace();
          DriverStation.reportError("PATH FAILED, CHECK PATH LOCATION", e.getStackTrace());
        }
        return t;
      }

    /*
        @param trajectory: the trajectory to follow
    */
    public static Command getPathweaverCommand(Trajectory trajectory) {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                Drivetrain.FEEDFORWARD,
                Drivetrain.KINEMATICS,
                10);

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    DrivetrainConstants.kMaxSpeedMPS/3,
                    DrivetrainConstants.kMaxAcceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Drivetrain.KINEMATICS)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
        
        // An example trajectory to follow.  All units in meters.
        RamseteCommand ramseteCommand =
            new RamseteCommand(
                trajectory,
                (()-> {return Drivetrain.ODOMETRY.getPoseMeters();}),
                new RamseteController(),
                Drivetrain.FEEDFORWARD,
                Drivetrain.KINEMATICS,
                Drivetrain::getWheelSpeeds,
                new PIDController(DrivetrainConstants.kPV, 0, 0),
                new PIDController(DrivetrainConstants.kPV, 0, 0),
                // RamseteCommand passes volts to the callback
                Drivetrain::setVoltages,
                Drivetrain.getInstance());
        drivetrain.resetEncoders();
        // Reset odometry to the starting pose of the trajectory.
        Drivetrain.ODOMETRY.resetPosition(navX.getRotation2d(), Drivetrain.getLeftEnc(), Drivetrain.getRightEnc(), trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return new InstantCommand(
            () -> Drivetrain.ODOMETRY.resetPosition(
                    navX.getRotation2d(), Drivetrain.getLeftEnc(), Drivetrain.getRightEnc(), trajectory.getInitialPose()
                )
            ).andThen(
            ramseteCommand.andThen(
                () -> Drivetrain.setOpenLoop(0, 0)
            )
        );
    }

     /**
     * Deadbands an input to [-1, -deadband], [deadband, 1], rescaling inputs to be
     * linear from (deadband, 0) to (1,1)
     * 
     * @param input    The input value to rescale
     * @param deadband The deadband
     * @return the input rescaled and to fit [-1, -deadband], [deadband, 1]
     */
    public static double deadbandX(double input, double deadband) {
        if(Math.abs(input) <= deadband) {
            return 0;
        } else if(Math.abs(input) == 1) {
            return input;
        } else {
            return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
        }
    }

    public static double getThrottle() {
        return -deadbandX(driverController.getLeftY(), Constants.DriverConstants.kJoystickDeadband);
    }

    public static double getOperatorLeftY() {
        return deadbandX(operatorController.getLeftY(), Constants.DriverConstants.kJoystickDeadband);
    }
    public static double getAltThrottle() {
        return -deadbandX(driverController.getRightY(), Constants.DriverConstants.kJoystickDeadband);
    }

    public static double getOperatorRightY() {
        return deadbandX(operatorController.getRightY(), Constants.DriverConstants.kJoystickDeadband);
    }
    public static double getTurn() {
        return deadbandX(driverController.getRightX(), Constants.DriverConstants.kJoystickDeadband);
    }

    public static double getLeftClimb() {
        return -deadbandX(operatorController.getLeftY(), Constants.DriverConstants.kJoystickDeadband);
    }

    public static double getRightClimb() {
        return -deadbandX(operatorController.getRightY(), Constants.DriverConstants.kJoystickDeadband);
    }

    public static double getOperatorLT() {
        return deadbandX(operatorController.getLeftTriggerAxis(), DriverConstants.kJoystickDeadband);
    }

    public static double getOperatorRT() {
        return deadbandX(operatorController.getRightTriggerAxis(), DriverConstants.kJoystickDeadband);
    }

    public static double getAbsoluteEncoder(){
        return getAbsoluteEncoder();
    }

    

    /*public static Color getColor() {
        return colorSensorV3.getColor();
    }     

    public int getProximity() {
        return colorSensorV3.getProximity();
    }*/

    /**
     * Set the LED mode on the Limelight
     * 
     * @param ledMode The mode to set the Limelight LEDs to
     */
    public void setLEDMode(LEDMode ledMode) {
        limelight.getEntry("ledMode").setNumber(ledMode.val);
    }
    /**
     * Sets the appearance of the Limelight camera stream
     * 
     * @param stream Stream mode to set the Limelight to
     */
    public void setStreamMode(StreamMode stream) {
        limelight.getEntry("stream").setNumber(stream.val);
    }
    /**
     * Sets Limelight vision pipeline
     * 
     * @param pipeline The pipeline to use
     */
    public void setPipeline(IntakeVisionPipeline pipeline) {
        limelight.getEntry("pipeline").setNumber(pipeline.val);
    }
    /**
     * Returns the horizontal offset between the target and the crosshair in degrees
     * 
     * @return the horizontal offset between the target and the crosshair in degrees
     */
    public static double getXOffset() {
        return -limelight.getEntry("tx").getDouble(0);
    }
    
    public static double getDistance() { // u need to do some math realquick
        double mountAngle = arm.getPosition(); //yessir
        double offsetAngle = limelight.getEntry("ty").getDouble(0.0);
        double angleGoalRads = (mountAngle + offsetAngle) * (Math.PI/180);
        return Units.InchesToMeters(VisionConstants.goalHeightInches - VisionConstants.limelightHeightInches)/(Math.tan(angleGoalRads)); //limelight h inches not constant
    }
    /**
     * Returns the vertical offset between the target and the crosshair in degrees
     * 
     * @return the vertical offset between the target and the crosshair in degrees
     */
    public static double getYOffset() {
        return -limelight.getEntry("ty").getDouble(0.0);
    }
    /**
     * Enum representing the different possible Limelight LED modes
     */
    public enum LEDMode {
        PIPELINE(0), OFF(1), BLINK(2), ON(3);

        public int val;

        LEDMode(int val) {
            this.val = val;
        }
    }

    /**
     * Enum representing the different possible Limelight stream modes
     */
    public enum StreamMode {
        SIDE_BY_SIDE(0), PIP_MAIN(1), PIP_SECONDARY(2);

        public int val;

        StreamMode(int val) {
            this.val = val;
        }
    }

    /**
     * Enum representing the different possible Limelight vision pipelines
     */
    public enum VisionPipeline {
        VISION(0), DRIVER(1);

        public int val;

        VisionPipeline(int val) {
            this.val = val;
        }
    }

    public enum IntakeVisionPipeline {
        RED(2), BLUE(1), ROBOT(0), DRIVER(3), INVALID(4);

        public int val;

        IntakeVisionPipeline(int val) {
            this.val = val;
        }
        
    }
    public enum ShooterVisionPipeline {
        ROBOT(0);

        public int val;

        ShooterVisionPipeline(int val) {
            this.val = val;
        }
        
    }
    public static IntakeVisionPipeline allianceToPipeline() {
        Alliance alliance = DriverStation.getAlliance();
        switch(alliance) {
            case Blue:
                return IntakeVisionPipeline.BLUE;
            case Red:
                return IntakeVisionPipeline.RED;
            default:
                return IntakeVisionPipeline.INVALID;
        }
    }
    /*public static PhotonPipelineResult getSnapshot() {
        return camera.getLatestResult();
    }*/
}