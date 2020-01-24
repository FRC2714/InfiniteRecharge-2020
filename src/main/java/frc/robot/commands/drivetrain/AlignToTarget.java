package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

import java.util.function.BooleanSupplier;

public class AlignToTarget extends PIDCommand {
    private Limelight limelight;
    private BooleanSupplier isButtonHeld;

    public AlignToTarget(Limelight limelight, Drivetrain drive, BooleanSupplier isButtonHeld) {
        super(
                new PIDController(DriveConstants.kAlignP,0, DriveConstants.kAlignD),
                // Close loop on heading
                limelight::getXAngleOffset,
                // set reference to target
                0,
                // pipe to turn robot
                output -> drive.arcadeDrive(0, output),
                // require drive
                drive
        );

        this.limelight = limelight;
        this.isButtonHeld = isButtonHeld;

        getController().enableContinuousInput(-180, 180);

        // TODO: set tolerances
        getController().setTolerance(.75);
    }


    @Override
    public boolean isFinished() {
        return
                getController().atSetpoint() || !limelight.targetVisible();
    }

}
