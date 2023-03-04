package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.Units;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveXMeters;
import frc.robot.commands.GridTrack;
import frc.robot.commands.TurnXDegrees;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/*
    Class to store autonomous sequences, including sequences such as intake
*/

public class Auto {
    public enum Selection {
        MOVE(1), OTHER(2);
        public int val;
        private Selection(int val) {
            this.val = val;
        }
    }

    public static Command getShootCommand() { //Drive up and shoot
        return new SequentialCommandGroup(
            new GridTrack()//,
            //new DriveXMeters(AutoConstants.hubXOffset, AutoConstants.DXMConstraints[0], AutoConstants.DXMConstraints[1]), 
            // new SmartShoot(RobotContainer.getDistance()).withTimeout(4) //TODO: adjust shooter velocity based on distance
        );
    }

    public static Command getBackupCommand() { //Back up and find new ball
        return new SequentialCommandGroup(
            // new DriveXMeters(-AutoConstants.backupDistance, AutoConstants.DXMConstraints[0], AutoConstants.DXMConstraints[1]),
            new TurnXDegrees(180, AutoConstants.TXDConstraints[0], AutoConstants.TXDConstraints[1])
        );
    }
}