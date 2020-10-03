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
                                new Pose2d(Units.feetToMeters(10.85), Units.feetToMeters(15.18), new Rotation2d().fromDegrees(-0.07)),
                                new Pose2d(Units.feetToMeters(19.25), Units.feetToMeters(8.03), new Rotation2d().fromDegrees(13.77)),
                                new Pose2d(Units.feetToMeters(22.00), Units.feetToMeters(9.19), new Rotation2d().fromDegrees(108.88)),
                                new Pose2d(Units.feetToMeters(19.97), Units.feetToMeters(13.83), new Rotation2d().fromDegrees(113.62))
                        ),
                        Units.feetToMeters(10), Units.feetToMeters(9), false
                );

        CustomRamseteCommand generatorToShot =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(19.71), Units.feetToMeters(14.20), new Rotation2d().fromDegrees(113.12)),
                                new Pose2d(Units.feetToMeters(15.24), Units.feetToMeters(13.81), new Rotation2d().fromDegrees(-19.99))
                        ),
                        Units.feetToMeters(10), Units.feetToMeters(9), true
                );

        addCommands(
                sequence(
                        new InstantCommand(() -> drivetrain.resetOdometry(baseLineToGenerator.getInitialPose())),
//                        new AlignToTarget(drivetrain, limelight),
                        baseLineToGenerator.andThen(() -> drivetrain.tankDriveVolts(0,0)),
                        generatorToShot.andThen(() -> drivetrain.tankDriveVolts(0,0))
//                        new AlignToTarget(drivetrain, limelight)
                )
        );

    }

}
