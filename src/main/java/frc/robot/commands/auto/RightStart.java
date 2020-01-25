package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.trajectories.CustomRamseteCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.RamseteGenerator;

import java.util.List;

public class RightStart extends SequentialCommandGroup {
    private static final Pose2d initialPose =
            new Pose2d(Units.feetToMeters(10.53), Units.feetToMeters(19.63), new Rotation2d().fromDegrees(0.00));


    public RightStart(Drivetrain drivetrain) {
        CustomRamseteCommand lineToTrench =
                RamseteGenerator.getRamseteCommand
                 (
                        drivetrain,
                        initialPose,
                        List.of(
                                new Translation2d(Units.feetToMeters(14.89), Units.feetToMeters(23.75))
                        ),

                        new Pose2d(Units.feetToMeters(25.68), Units.feetToMeters(24.61), new Rotation2d().fromDegrees(0.00)),
                        Units.feetToMeters(13.3), Units.feetToMeters(8.75), false
                );

        addCommands(
                sequence(
                        new InstantCommand(() -> drivetrain.resetOdometry(initialPose)),
                        lineToTrench.andThen(()->drivetrain.tankDriveVolts(0,0))
                )
        );

    }

}
