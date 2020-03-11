package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.AlignToTarget;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.conveyor.AutomaticShoot;
import frc.robot.subsystems.*;
import frc.robot.utils.CustomRamseteCommand;
import frc.robot.utils.RamseteGenerator;

import java.util.List;

public class BallStealAuto extends SequentialCommandGroup {

    public BallStealAuto(Drivetrain drivetrain, Intake intake, Conveyor conveyor, Shooter shooter, Limelight limelight){
        CustomRamseteCommand baselineToStealBalls =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(12.07), Units.feetToMeters(2.15), new Rotation2d().fromDegrees(0.29)),
                                new Pose2d(Units.feetToMeters(20.39), Units.feetToMeters(2.15), new Rotation2d().fromDegrees(0.00))
                        ),
                        Units.feetToMeters(13), Units.feetToMeters(7), false
                );

        CustomRamseteCommand reverseBallsStealToShotSetup =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(21.18), Units.feetToMeters(1.89), new Rotation2d().fromDegrees(40.67)),
                                new Pose2d(Units.feetToMeters(18.90), Units.feetToMeters(3.76), new Rotation2d().fromDegrees(-90.11)),
                                new Pose2d(Units.feetToMeters(16.82), Units.feetToMeters(12.30), new Rotation2d().fromDegrees(-27.01))
                        ),
                        Units.feetToMeters(13), Units.feetToMeters(8), true
                );

        addCommands(
                sequence(
                        new InstantCommand(() -> drivetrain.resetOdometry(baselineToStealBalls.getInitialPose())),
                        deadline(
                                baselineToStealBalls,
                                new AutoIntake(shooter,intake,conveyor, AutoIntake.IntakeType.INTAKE)
                        ),
                        reverseBallsStealToShotSetup.andThen(() -> drivetrain.tankDriveVolts(0,0)),
                        new AlignToTarget(drivetrain, limelight).withTimeout(1.5),
                        new AutomaticShoot(shooter,conveyor,intake, 2050, true, 3).withTimeout(3)
                )
        );

    }

}
