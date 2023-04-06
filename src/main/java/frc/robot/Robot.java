// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.Auto;
import frc.robot.commands.DriveXMeters;
import frc.robot.commands.TurnXDegrees;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autoSelected;
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private RobotContainer robot;
  private PowerDistribution pdp = new PowerDistribution();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    robot = RobotContainer.getInstance();
    pdp.clearStickyFaults();
    // m_chooser.setDefaultOption("Default Auto (Move Arm)",
    //     RobotContainer.getAutonomousCommand(Auto.Selection.MOVEWRIST));
    SmartDashboard.putData("Auto choices", m_chooser);
    // INITIALIZE TO STOW POSITION
    RobotContainer.wrist.setWristPositionAuto(Intake.ScorePos.STOW);
    RobotContainer.intake.set(0);
    RobotContainer.arm.setArmPositionAuto(Intake.ScorePos.STOW);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Port 21 Current", pdp.getCurrent(21));
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    pdp.clearStickyFaults();
    // m_chooser.addOption("LOW CONE BACKUP", Auto.lowConeBackup());
    // m_chooser.addOption("HIGH CONE BACKUP", Auto.highConeBackup());
    // CommandScheduler.getInstance().schedule(m_chooser.getSelected());

    // System.out.println("Auto selected: " + m_autoSelected);
    // RobotContainer.getAutonomousCommand(Auto.Selection.MOVEWRIST);

    // CommandScheduler.getInstance().schedule(RobotContainer.getPathweaverCommand(RobotContainer.smallTraj));
    CommandScheduler.getInstance().schedule(new DriveXMeters(1., 1., 0.5));
    // MAIN AUTO
    // CommandScheduler.getInstance().schedule(
    //   new SequentialCommandGroup(
    //     new InstantCommand(() -> RobotContainer.wrist.setWristPositionAuto(Intake.ScorePos.STOW), RobotContainer.wrist),
    //     new InstantCommand(() -> RobotContainer.arm.setArmPositionAuto(Intake.ScorePos.STOW), RobotContainer.arm),
    //     new RunCommand(() -> RobotContainer.intake.set(-0.9), RobotContainer.intake).withTimeout(1),
    //     new WaitCommand(.5),
    //     new RunCommand(() -> Drivetrain.setOpenLoop(-0.25, -0.25), RobotContainer.drivetrain).withTimeout(2.2),
    //     new RunCommand(() -> RobotContainer.intake.set(0)).withTimeout(1))
    //   );

    // CommandScheduler.getInstance().schedule(Auto.highConeBackup());

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    // case kCustomAuto:
    // // Put custom auto code here
    // break;
    // case kDefaultAuto:
    // default:
    // // Put default auto code here
    // break;
    // }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // CommandScheduler.getInstance().schedule(new TurnXDegrees(180, Math.PI, Math.PI/2.));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("LTval", RobotContainer.getOperatorLT());
    SmartDashboard.putNumber("RTval", RobotContainer.getOperatorRT());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

}
