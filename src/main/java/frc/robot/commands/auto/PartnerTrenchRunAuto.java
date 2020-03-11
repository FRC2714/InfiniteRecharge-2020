package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.conveyor.AutomaticShoot;
import frc.robot.commands.drivetrain.AlignToTarget;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.subsystems.*;
import frc.robot.utils.CustomRamseteCommand;
import frc.robot.utils.RamseteGenerator;

import java.util.List;

public class PartnerTrenchRunAuto extends SequentialCommandGroup {

    public PartnerTrenchRunAuto(Drivetrain drivetrain, Intake intake, Conveyor conveyor, Shooter shooter, Limelight limelight){

        CustomRamseteCommand pushBackRobot =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(11.533), Units.feetToMeters(18.66), new Rotation2d().fromDegrees(0.00)),
                                new Pose2d(Units.feetToMeters(9.533), Units.feetToMeters(18.66), new Rotation2d().fromDegrees(0.00))
                                ),
                        Units.feetToMeters(6), Units.feetToMeters(7), true
                );

        CustomRamseteCommand quinticLineToTrench =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(9.533), Units.feetToMeters(18.66), new Rotation2d().fromDegrees(0.00)),
                                new Pose2d(Units.feetToMeters(16), Units.feetToMeters(23.8), new Rotation2d().fromDegrees(0.00)), //24.76
                                new Pose2d(Units.feetToMeters(27.5), Units.feetToMeters(24.2), new Rotation2d().fromDegrees(0.00)) //24.676
                        ),
                        Units.feetToMeters(5.5), Units.feetToMeters(7), false
                );

        addCommands(
                sequence(
                        new InstantCommand(() -> drivetrain.resetOdometry(pushBackRobot.getInitialPose())),
                        deadline(
                                new AutomaticShoot(shooter, conveyor, intake, 3000, true, 3).withTimeout(4.7),
                                new AlignToTarget(drivetrain, limelight, true).withTimeout(1)
                        ),
                        pushBackRobot.andThen(() -> drivetrain.tankDriveVolts(0,0)),
                        deadline(
                                quinticLineToTrench,
                                new AutoIntake(shooter, intake, conveyor, AutoIntake.IntakeType.INTAKE)
                        ),
                        deadline(
                                new AlignToTarget(drivetrain, limelight, true).withTimeout(3),
                                new AutoIntake(shooter, intake, conveyor, AutoIntake.IntakeType.INTAKE)
                        ),
                        deadline(
                                new AutomaticShoot(shooter, conveyor, intake, 2500, false, 3), //was 2300
                                new AlignToTarget(drivetrain, limelight, false)
                        )
                )

        );
    }

}
