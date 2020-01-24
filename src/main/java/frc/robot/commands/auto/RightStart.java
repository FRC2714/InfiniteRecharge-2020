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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.trajectories.CustomRamseteCommand;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

public class RightStart extends SequentialCommandGroup {

    public RightStart(Drivetrain drivetrain) {

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

        Trajectory mainSpline = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(10.854, 19.631, new Rotation2d().fromDegrees(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                        new Translation2d(Units.feetToMeters(18.141), Units.feetToMeters(23.936))
                ),
                new Pose2d(Units.feetToMeters(27.571), Units.feetToMeters(24.501), new Rotation2d().fromDegrees(0)),
                // Pass config
                config
        );

        CustomRamseteCommand forwardSpline = new CustomRamseteCommand(
                mainSpline,
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
                        new InstantCommand(() -> drivetrain.resetOdometry(mainSpline.getInitialPose())),
                        forwardSpline.andThen(()->drivetrain.tankDriveVolts(0,0))
                )
        );

    }

}
