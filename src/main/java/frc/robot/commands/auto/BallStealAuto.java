package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.AlignToTarget;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.CustomRamseteCommand;
import frc.robot.utils.RamseteGenerator;

import java.util.List;

public class BallStealAuto extends SequentialCommandGroup {

    public BallStealAuto(Drivetrain drivetrain, Limelight limelight){
        CustomRamseteCommand baselineToStealBalls =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(11.98), Units.feetToMeters(2.20), new Rotation2d().fromDegrees(0.00)),
                                new Pose2d(Units.feetToMeters(20.51), Units.feetToMeters(2.20), new Rotation2d().fromDegrees(0.00))
                        ),
                        Units.feetToMeters(13), Units.feetToMeters(10), false
                );

        CustomRamseteCommand reverseBallsStealToShotSetup =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(20.50), Units.feetToMeters(2.25), new Rotation2d().fromDegrees(0.00)),
                                new Pose2d(Units.feetToMeters(15.96), Units.feetToMeters(13.57), new Rotation2d().fromDegrees(-28.02))
                        ),
                        Units.feetToMeters(13), Units.feetToMeters(8), true
                );

        addCommands(
                sequence(
                        new InstantCommand(() -> drivetrain.resetOdometry(baselineToStealBalls.getInitialPose())),
                        baselineToStealBalls.andThen(() -> drivetrain.tankDriveVolts(0,0)),
                        reverseBallsStealToShotSetup.andThen(() -> drivetrain.tankDriveVolts(0,0)),
                        new AlignToTarget(limelight, drivetrain)
                )
        );

    }

}
