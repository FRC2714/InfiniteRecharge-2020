package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.AlignToTarget;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.shooter.AutomaticShooter;
import frc.robot.subsystems.*;
import frc.robot.utils.CustomRamseteCommand;
import frc.robot.utils.RamseteGenerator;

import java.util.List;

public class SideTrenchRun extends SequentialCommandGroup {

    public SideTrenchRun(Drivetrain drivetrain, Intake intake, Conveyor conveyor, Shooter shooter, Limelight limelight){
        CustomRamseteCommand quinticStraightLineToTrench =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(11.89), Units.feetToMeters(24.65), new Rotation2d().fromDegrees(0)),
                                new Pose2d(Units.feetToMeters(15.46), Units.feetToMeters(24.68), new Rotation2d().fromDegrees(0))
                        ),
                        Units.feetToMeters(13), Units.feetToMeters(8), false
                );

        CustomRamseteCommand quinticPickupBalls =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(15.46), Units.feetToMeters(24.68), new Rotation2d().fromDegrees(0.50)),
                                new Pose2d(Units.feetToMeters(26.03), Units.feetToMeters(24.65), new Rotation2d().fromDegrees(0.34))
                        ),
                        Units.feetToMeters(5), Units.feetToMeters(5), false
                );

        addCommands(
                sequence(
                        new InstantCommand(() -> drivetrain.resetOdometry(quinticStraightLineToTrench.getInitialPose())),
                        quinticStraightLineToTrench.andThen(() -> drivetrain.tankDriveVolts(0,0)),
                        new AlignToTarget(drivetrain, limelight),
                        new AutomaticShooter(shooter,conveyor,intake, 2000, true,3).withTimeout(5),
                        new TurnToAngle(drivetrain, 0),
                        deadline(
                                quinticPickupBalls,
                                new AutoIntake(shooter,intake,conveyor, AutoIntake.IntakeType.INTAKE)
                        ),
                        new AlignToTarget(drivetrain, limelight),
                        new AutomaticShooter(shooter,conveyor,intake, 2350, false,3).withTimeout(5)
                )
        );
    }

}
