package frc.robot.commands.drivetrain.trajectories;

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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

public class FollowTrajectory extends CommandBase {

    private Drivetrain drivetrain;

    private DifferentialDriveVoltageConstraint voltageConstraint;
    private TrajectoryConfig config;
    private RamseteCommand ramseteCommand;

    double velocity, acceleration;
    boolean isReversed;

    Pose2d startPose, endPose;
    List<Translation2d> internalPoints;

    public FollowTrajectory(Drivetrain drivetrain,
                            Pose2d startPose,
                            List<Translation2d> internalPoints,
                            Pose2d endPose,
                            double velocity, double acceleration, boolean isReversed) {
        this.drivetrain = drivetrain;
        this.startPose = startPose;
        this.endPose = endPose;
        this.internalPoints = internalPoints;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.isReversed = isReversed;
    }

    @Override
    public void initialize() {
        System.out.println("Follow trajectory initialized");
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

        CentripetalAccelerationConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(1.75);

        config =
                new TrajectoryConfig(Units.feetToMeters(10), Units.feetToMeters(6.5))
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(drivetrain.getKinematics())
                        // Apply the voltage constraint
                        .addConstraint(voltageConstraint)
                        .addConstraint(centripetalAccelerationConstraint);

        config.setReversed(isReversed);
        assert config.isReversed() == isReversed : "Follow Path reverse not working!";

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startPose,
                // Pass through these two interior waypoints, making an 's' curve path
                internalPoints,

                // Ending point
                endPose,
                // Pass config
                config
        );

        this.ramseteCommand = new RamseteCommand(
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

        ramseteCommand.schedule();
        System.out.println("----STARTING PATH TRACKING----");
        System.out.println("Initial X Position " + Units.metersToFeet(drivetrain.getPose().getTranslation().getX()) + " | Initial Y Position " +
                Units.metersToFeet(drivetrain.getPose().getTranslation().getY()) +
                " | Initial Heading " + drivetrain.getPose().getRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
        return ramseteCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("FOLOW TRAJ FINISHED *****&DADA^^^&");
        System.out.println();
        System.out.println("Final X Position " + Units.metersToFeet(drivetrain.getPose().getTranslation().getX()) + " | Final Y Position " +
                Units.metersToFeet(drivetrain.getPose().getTranslation().getY()) +
                " | Final Heading " + drivetrain.getPose().getRotation().getDegrees());
        System.out.println("---ENDED PATH TRACKING---");
        System.out.println();
    }
}
