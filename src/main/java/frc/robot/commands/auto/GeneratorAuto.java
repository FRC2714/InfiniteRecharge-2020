package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.AlignToTarget;
import frc.robot.utils.CustomRamseteCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.RamseteGenerator;

import java.util.List;

public class GeneratorAuto extends SequentialCommandGroup {

    public GeneratorAuto(Drivetrain drivetrain, Limelight limelight) {
        CustomRamseteCommand baseLineToGenerator =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(12.28), Units.feetToMeters(15.18), new Rotation2d().fromDegrees(0.00)),
                                new Pose2d(Units.feetToMeters(18.32), Units.feetToMeters(13.38), new Rotation2d().fromDegrees(22.81))
                        ),
                        Units.feetToMeters(8), Units.feetToMeters(6), false
                );

        CustomRamseteCommand reverseBaseLineToGenerator =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(18.32), Units.feetToMeters(13.38), new Rotation2d().fromDegrees(22.81)),
                                new Pose2d(Units.feetToMeters(12.28), Units.feetToMeters(15.18), new Rotation2d().fromDegrees(0.00))
                        ),
                        Units.feetToMeters(8), Units.feetToMeters(6), true
                );

        addCommands(
                sequence(
                        new InstantCommand(() -> drivetrain.resetOdometry(baseLineToGenerator.getInitialPose())),
                        new AlignToTarget(drivetrain, limelight),
                        baseLineToGenerator.andThen(() -> drivetrain.tankDriveVolts(0, 0)),
                        reverseBaseLineToGenerator.andThen(() -> drivetrain.tankDriveVolts(0, 0)),
                        new AlignToTarget(drivetrain, limelight)
                )
        );

    }

}
