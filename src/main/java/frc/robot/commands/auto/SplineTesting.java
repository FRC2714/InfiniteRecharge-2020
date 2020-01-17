package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.trajectories.FollowTrajectory;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

public class SplineTesting extends SequentialCommandGroup {

    public SplineTesting(Drivetrain drivetrain) {
        addCommands(

                new InstantCommand(drivetrain::resetAll),

                new FollowTrajectory(
                        drivetrain,
                        new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d().fromDegrees(0)),
                        List.of(
                                new Translation2d(Units.feetToMeters(5), Units.feetToMeters(-2.5))
                        ),
                        new Pose2d(Units.feetToMeters(10), Units.feetToMeters(-4.5), new Rotation2d().fromDegrees(0)),
                        3.3, 2, false
                ),

                new FollowTrajectory(
                        drivetrain,
                        new Pose2d(Units.feetToMeters(10), Units.feetToMeters(-4.5), new Rotation2d().fromDegrees(0)),
                        List.of(
                                new Translation2d(Units.feetToMeters(5), Units.feetToMeters(-2.5))
                        ),
                        new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d().fromDegrees(0)),
                        3.3, 2, true
                ),

                new InstantCommand(() -> System.out.println("SETTING TANK TO ZERO"))

        );

    }

}
