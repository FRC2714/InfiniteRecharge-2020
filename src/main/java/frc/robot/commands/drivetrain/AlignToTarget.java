package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AlignToTarget extends PIDCommand {

    public AlignToTarget(Limelight limelight, Drivetrain drive) {
        super(
                new PIDController(.12,0, .03),
                // Close loop on heading
                limelight::getXAngleOffset,
                // set reference to target
                0,
                // pipe to turn robot
                output -> drive.arcadeDrive(0, output),
                // require drive
                drive
        );

        getController().enableContinuousInput(-180, 180);

        // TODO: set tolerances
        getController().setTolerance(.75);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}
