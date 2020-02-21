package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.AlignToTarget;
import frc.robot.utils.CustomRamseteCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.RamseteGenerator;

import java.util.List;

public class SplineTesting extends SequentialCommandGroup {


    public SplineTesting(Drivetrain drivetrain, Limelight limelight) {

        CustomRamseteCommand simpleCurve =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(9.50), Units.feetToMeters(19.44), new Rotation2d().fromDegrees(0.00)),
                                new Pose2d(Units.feetToMeters(18.53), Units.feetToMeters(24.65), new Rotation2d().fromDegrees(0.00)),
                                new Pose2d(Units.feetToMeters(26.75), Units.feetToMeters(24.65), new Rotation2d().fromDegrees(0.00))
                        ),
                        Units.feetToMeters(13), Units.feetToMeters(8), false
                );


        addCommands(
                sequence(
                        new InstantCommand(() -> drivetrain.resetOdometry(simpleCurve.getInitialPose())),
                        simpleCurve.andThen(() -> drivetrain.tankDriveVolts(0, 0))
                )
        );

    }

}