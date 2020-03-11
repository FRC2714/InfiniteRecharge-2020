package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.conveyor.AutomaticShoot;
import frc.robot.commands.drivetrain.AlignToTarget;
import frc.robot.subsystems.*;
import frc.robot.utils.CustomRamseteCommand;
import frc.robot.utils.RamseteGenerator;

import java.util.List;

public class WaitShootAuton extends SequentialCommandGroup {

    public WaitShootAuton(Drivetrain drivetrain, Intake intake, Conveyor conveyor, Shooter shooter, Limelight limelight, boolean driveBackwards){

        CustomRamseteCommand moveOutOfLine =
                RamseteGenerator.getRamseteCommand(
                        drivetrain,
                        List.of(
                                new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d().fromDegrees(0.00)),
                                new Pose2d(Units.feetToMeters((driveBackwards) ? -5 : 5), Units.feetToMeters(0), new Rotation2d().fromDegrees(0.00))
                        ),
                        Units.feetToMeters(5), Units.feetToMeters(7), driveBackwards
                );

        addCommands(
                sequence(
                        new InstantCommand(() -> drivetrain.resetOdometry(moveOutOfLine.getInitialPose())),
                        new AlignToTarget(drivetrain,limelight).withTimeout(2.5),
                        new AutomaticShoot(shooter,conveyor,intake,shooter.getTargetRpm(), false, 3).
                                withTimeout(6),
                        moveOutOfLine
                )
        );
    }

}
