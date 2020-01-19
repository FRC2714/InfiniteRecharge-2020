package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends PIDCommand {

    public TurnToAngle(double targetAngleDegrees, Drivetrain drive) {
        super(
                new PIDController(DriveConstants.kHeadingP, DriveConstants.kHeadingI, DriveConstants.kHeadingD),
                // Close loop on heading
                drive::getHeading,
                // set reference to target
                targetAngleDegrees,
                // pipe to turn robot
                output -> drive.arcadeDrive(0, output),
                // require drive
                drive
        );

        getController().enableContinuousInput(-180,180);

        // TODO: set tolerances
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}
