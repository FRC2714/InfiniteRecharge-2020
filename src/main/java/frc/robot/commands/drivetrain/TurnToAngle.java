package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends ProfiledPIDCommand {

    public TurnToAngle(Drivetrain drive, double targetAngleDegrees) {
        super(
                new ProfiledPIDController(DriveConstants.kAlignP, 0, DriveConstants.kAlignD,
                        new TrapezoidProfile.Constraints(100, 300)),
                // Close loop on heading
                () -> drive.getHeading() * -1,
                // set reference to target
                targetAngleDegrees,
                // pipe to turn robot
                (output, setpoint) -> drive.arcadeDrive(0, Math.signum(output) * 0.2 + output),
                // require drive
                drive
        );

        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(.75, 4);
        // TODO: set tolerances
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }

}
