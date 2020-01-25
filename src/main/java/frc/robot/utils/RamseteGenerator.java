package frc.robot.utils;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.trajectories.CustomRamseteCommand;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

public class RamseteGenerator {
    private static TrajectoryConfig getConfig(double maxVel, double maxA, boolean isReversed) {
        return new TrajectoryConfig(Units.feetToMeters(13.3), Units.feetToMeters(8.75))
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                Constants.DriveConstants.kStatic,
                                Constants.DriveConstants.kV,
                                Constants.DriveConstants.kA
                        ),
                        Constants.DriveConstants.kDriveKinematics,
                        10
                ))
                .addConstraint(
                        new CentripetalAccelerationConstraint(Units.feetToMeters(5.4))
                ).setReversed(isReversed);
    }

    public static CustomRamseteCommand getRamseteCommand(Drivetrain drivetrain,
                                                         Pose2d startPose,
                                                         List<Translation2d> internalPoints,
                                                         Pose2d endPose,
                                                         double velocity, double acceleration, boolean isReversed) {

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startPose,
                // Pass through these two interior waypoints, making an 's' curve path
                internalPoints,

                // Ending point
                endPose,
                // Pass config
                getConfig(velocity, acceleration, isReversed)
        );

        return new CustomRamseteCommand(
                trajectory,
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
    }

}
