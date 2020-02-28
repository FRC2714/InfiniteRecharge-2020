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

public class NormalTrenchRunAuto extends SequentialCommandGroup {

    public NormalTrenchRunAuto(Drivetrain drivetrain, Intake intake, Conveyor conveyor, Shooter shooter, Limelight limelight) {
        CustomRamseteCommand quinticLineToTrench =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(9.533), Units.feetToMeters(18.66), new Rotation2d().fromDegrees(0.00)),
                                new Pose2d(Units.feetToMeters(16), Units.feetToMeters(24.72), new Rotation2d().fromDegrees(0.00)),
                                new Pose2d(Units.feetToMeters(27.3), Units.feetToMeters(24.76), new Rotation2d().fromDegrees(0.00))
                        ),
                        Units.feetToMeters(5 ), Units.feetToMeters(7), false
                );
        addCommands(
                sequence(
                        deadline(
                                new AutomaticShoot(shooter, conveyor, intake, 3000, true, 3).withTimeout(4.7),
                                new AlignToTarget(drivetrain, limelight, true)
                        ),
                        new InstantCommand(() -> drivetrain.resetOdometry(quinticLineToTrench.getInitialPose())),
                        deadline(
                                    quinticLineToTrench,
                                    new AutoIntake(shooter, intake, conveyor, AutoIntake.IntakeType.INTAKE)
                        ),
                        deadline(
                                new AlignToTarget(drivetrain, limelight, true).withTimeout(1.5),
                                new AutoIntake(shooter, intake, conveyor, AutoIntake.IntakeType.INTAKE)
                        ),
                        // was 2300
                        deadline(
                                new AutomaticShoot(shooter, conveyor, intake, 2300, false, 3),
                                new AlignToTarget(drivetrain, limelight, false)
                        )
                )

        );

    }

}
