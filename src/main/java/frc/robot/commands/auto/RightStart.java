package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.AlignToTarget;
import frc.robot.commands.drivetrain.trajectories.CustomRamseteCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.RamseteGenerator;

import java.util.List;

public class RightStart extends SequentialCommandGroup {

    public RightStart(Drivetrain drivetrain, Limelight limelight) {

        CustomRamseteCommand quinticLineToTrench =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(10.82), Units.feetToMeters(19.40), new Rotation2d().fromDegrees(0.00)),
                                new Pose2d(Units.feetToMeters(17.62), Units.feetToMeters(24.60), new Rotation2d().fromDegrees(0.00)),
                                new Pose2d(Units.feetToMeters(25.88), Units.feetToMeters(24.65), new Rotation2d().fromDegrees(0.00))
                        ),
                        Units.feetToMeters(13), Units.feetToMeters(8.5), false
                );

        addCommands(
                sequence(
                        new InstantCommand(() -> drivetrain.resetOdometry(quinticLineToTrench.getInitialPose())),
                        quinticLineToTrench.andThen(() -> drivetrain.tankDriveVolts(0, 0)),
                        new AlignToTarget(limelight, drivetrain)
                )
        );

    }

}
