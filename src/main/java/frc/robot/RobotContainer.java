/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CustomRamseteCommand;
import frc.robot.commands.DriverControl;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Logger;

import java.util.List;

import static frc.robot.Constants.DriveConstants.kRamseteB;
import static frc.robot.Constants.DriveConstants.kRamseteZeta;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Drivetrain drivetrain = new Drivetrain();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public static Joystick driverStick = new Joystick(0);

  private DriverControl driverControlCommand = new DriverControl(
          drivetrain,
          () -> driverStick.getRawAxis(1),
          () -> driverStick.getRawAxis(4)
  );

  private Logger<Subsystem> subsystemLogger = new Logger<>();


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public Command getDriverControlCommand() {
    return driverControlCommand;
  }

  public Command getRamseteCommand() {
    drivetrain.resetAll();
    DifferentialDriveVoltageConstraint voltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                            Constants.DriveConstants.kStatic,
                            Constants.DriveConstants.kV,
                            Constants.DriveConstants.kA
                    ),
                    drivetrain.getKinematics(),
                    12
            );
    TrajectoryConfig config =
            new TrajectoryConfig(4, 2)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(drivetrain.getKinematics())
                    // Apply the voltage constraint
                    .addConstraint(voltageConstraint);
    TrajectoryConfig reverseConfig =
            new TrajectoryConfig(4, 2)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(drivetrain.getKinematics())
                    // Apply the voltage constraint
                    .addConstraint(voltageConstraint)
                    .setReversed(true);

    /*
    Trajectory simpleSCurve = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d().fromDegrees(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                    new Translation2d(Units.feetToMeters(6), Units.feetToMeters(0)),
                    new Translation2d(Units.feetToMeters(10), Units.feetToMeters(4))

            ),
            new Pose2d(Units.feetToMeters(), Units.feetToMeters(), new Rotation2d().fromDegrees(30)),
            // Pass config
            config
    );*/

    /*
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
    );*/

    Trajectory simpleSCurve = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d().fromDegrees(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                    new Translation2d(Units.feetToMeters(5), Units.feetToMeters(-2.5))
            ),
            new Pose2d(Units.feetToMeters(10), Units.feetToMeters(-4.5), new Rotation2d().fromDegrees(0)),
            // Pass config
             config
    );

    Trajectory reverseSimpleSCurve = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(10, -4.5, new Rotation2d().fromDegrees(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                    new Translation2d(Units.feetToMeters(5), Units.feetToMeters(-2.5))
            ),
            new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d().fromDegrees(0)),
            // Pass config
            reverseConfig
    );

    CustomRamseteCommand ramseteCommand = new CustomRamseteCommand(
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
            new PIDController(Constants.DriveConstants.kDriveP,0,Constants.DriveConstants.kDriveD),
            new PIDController(Constants.DriveConstants.kDriveP,0,Constants.DriveConstants.kDriveD),
            drivetrain::tankDriveVolts,
            drivetrain
    );

    CustomRamseteCommand ramseteCommand2 = new CustomRamseteCommand(
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
            new PIDController(Constants.DriveConstants.kDriveP,0,Constants.DriveConstants.kDriveD),
            new PIDController(Constants.DriveConstants.kDriveP,0,Constants.DriveConstants.kDriveD),
            drivetrain::tankDriveVolts,
            drivetrain
    );

    return ramseteCommand.andThen(ramseteCommand2.andThen(() -> drivetrain.tankDriveVolts(0,0)));
  }


}
