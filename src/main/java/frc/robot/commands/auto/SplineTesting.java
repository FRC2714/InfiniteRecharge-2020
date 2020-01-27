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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.AlignToTarget;
import frc.robot.commands.drivetrain.trajectories.CustomRamseteCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

import java.util.List;

public class SplineTesting extends SequentialCommandGroup {


    public SplineTesting(Drivetrain drivetrain, Limelight limelight) {

        DifferentialDriveVoltageConstraint voltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                Constants.DriveConstants.kStatic,
                                Constants.DriveConstants.kV,
                                Constants.DriveConstants.kA
                        ),
                        drivetrain.getKinematics(),
                        10
                );

        CentripetalAccelerationConstraint centripetalAccelerationConstraint =
                new CentripetalAccelerationConstraint(Units.feetToMeters(5.4));

        TrajectoryConfig config =
                new TrajectoryConfig(Units.feetToMeters(13.3), Units.feetToMeters(8.75))
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(drivetrain.getKinematics())
                        // Apply the voltage constraint
                        .addConstraint(voltageConstraint)
                        .addConstraint(centripetalAccelerationConstraint);

        Trajectory simpleSCurve = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d().fromDegrees(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                        new Translation2d(Units.feetToMeters(7), Units.feetToMeters(-2.5))
                ),
                new Pose2d(Units.feetToMeters(13), Units.feetToMeters(-4.5), new Rotation2d().fromDegrees(30)),
                // Pass config
                config
        );

        Trajectory reverseSimpleSCurve = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(Units.feetToMeters(13), Units.feetToMeters(-4.5), new Rotation2d().fromDegrees(30)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                        new Translation2d(Units.feetToMeters(7), Units.feetToMeters(-2.5))
                ),
                new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d().fromDegrees(0)),
                // Pass config
                config.setReversed(true)
        );

        CustomRamseteCommand forwardSpline = new CustomRamseteCommand(
                simpleSCurve,
                drivetrain::getPose,
                new RamseteController(Constants.DriveConstants.kRamseteB, Constants.DriveConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        Constants.DriveConstants.kStatic,
                        Constants.DriveConstants.kV,
                        Constants.DriveConstants.kA
                ),
                drivetrain.getKinematics(),
                drivetrain::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kDriveP, 0, Constants.DriveConstants.kDriveD),
                new PIDController(Constants.DriveConstants.kDriveP, 0, Constants.DriveConstants.kDriveD),
                drivetrain::tankDriveVolts,
                drivetrain
        );

        CustomRamseteCommand reverseSpline = new CustomRamseteCommand(
                reverseSimpleSCurve,
                drivetrain::getPose,
                new RamseteController(Constants.DriveConstants.kRamseteB, Constants.DriveConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        Constants.DriveConstants.kStatic,
                        Constants.DriveConstants.kV,
                        Constants.DriveConstants.kA
                ),
                drivetrain.getKinematics(),
                drivetrain::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kDriveP, 0, Constants.DriveConstants.kDriveD),
                new PIDController(Constants.DriveConstants.kDriveP, 0, Constants.DriveConstants.kDriveD),
                drivetrain::tankDriveVolts,
                drivetrain
        );


        addCommands(
                sequence(
                        forwardSpline,
                        reverseSpline.andThen(() -> drivetrain.tankDriveVolts(0, 0)),
                        new AlignToTarget(limelight, drivetrain)
                )
        );

    }

}