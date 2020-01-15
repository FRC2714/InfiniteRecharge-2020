package frc.robot.commands.drivetrain.trajectories;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
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

    public FollowTrajectory(Drivetrain drivetrain,
                            Pose2d startPose,
                            List<Translation2d> internalPoints,
                            Pose2d endPose,
                            double velocity, double acceleration, boolean isReversed){
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;

        config = new TrajectoryConfig(velocity, acceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(drivetrain.getKinematics())
                // Apply the voltage constraint
                .addConstraint(voltageConstraint);

        config.setReversed(isReversed);
        assert config.isReversed() == isReversed : "Follow Path reverse not working!";

        voltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        Constants.DriveConstants.kStatic,
                        Constants.DriveConstants.kV,
                        Constants.DriveConstants.kA
                ),
                drivetrain.getKinematics(),
                12
        );

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
                new PIDController(Constants.DriveConstants.kDriveP,0,Constants.DriveConstants.kDriveD),
                new PIDController(Constants.DriveConstants.kDriveP,0,Constants.DriveConstants.kDriveD),
                drivetrain::tankDriveVolts,
                drivetrain
        );

    }

    @Override
    public void initialize() {
        ramseteCommand.initialize();
        ramseteCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return ramseteCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println();
        System.out.printf("Final X Position %.2f | Final Y Position %.2f | Final Heading %.2f \n",
                drivetrain.getPose().getTranslation().getX(),
                drivetrain.getPose().getTranslation().getY(),
                drivetrain.getPose().getRotation().getDegrees());
        System.out.println("---ENDED FORWARD PATH TRACKING---");
        System.out.println();
    }
}
