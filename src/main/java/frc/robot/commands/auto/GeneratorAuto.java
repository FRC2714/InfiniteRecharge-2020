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

    public GeneratorAuto(Drivetrain drivetrain, Limelight limelight){
        CustomRamseteCommand baseLineToGenerator =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(12.14), Units.feetToMeters(15.59), Rotation2d.fromDegrees(0.00)),
                                new Pose2d(Units.feetToMeters(20.53), Units.feetToMeters(18.10), Rotation2d.fromDegrees(-70))
                        ),
                        Units.feetToMeters(13), Units.feetToMeters(8), false
                );

        CustomRamseteCommand reverseBaseLineToGenerator =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(20.53), Units.feetToMeters(18.10), Rotation2d.fromDegrees(-70)),
                                new Pose2d(Units.feetToMeters(12.14), Units.feetToMeters(15.59), Rotation2d.fromDegrees(0.00))
                        ),
                        Units.feetToMeters(13), Units.feetToMeters(8), true
                );

        addCommands(
                sequence(
                        new InstantCommand(() -> drivetrain.resetOdometry(baseLineToGenerator.getInitialPose())),
                        new AlignToTarget(drivetrain, limelight),
                        baseLineToGenerator.andThen(() -> drivetrain.tankDriveVolts(0,0)),
                        reverseBaseLineToGenerator.andThen(() -> drivetrain.tankDriveVolts(0,0)),
                        new AlignToTarget(drivetrain, limelight)
                )
        );

    }

}
