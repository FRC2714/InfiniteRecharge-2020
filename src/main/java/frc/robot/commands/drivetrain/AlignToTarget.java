package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

import java.util.function.BooleanSupplier;

public class AlignToTarget extends ProfiledPIDCommand {
    private Limelight limelight;
    private Drivetrain drivetrain;
    public AlignToTarget(Limelight limelight, Drivetrain drive) {
        super(
                new ProfiledPIDController(DriveConstants.kAlignP,0, DriveConstants.kAlignD,
                        new TrapezoidProfile.Constraints(100, 300)),
                // Close loop on heading
                limelight::getXAngleOffset,
                // set reference to target
                0,
                // pipe to turn robot
                (output, setpoint) -> drive.arcadeDrive(0, output),
                // require drive
                drive
        );

        this.limelight = limelight;
        this.drivetrain = drive;
        getController().enableContinuousInput(-180, 180);

        // TODO: set tolerances
        getController().setTolerance(.75, 4);
    }


    @Override
    public boolean isFinished() {
        return
                getController().atGoal() || !limelight.targetVisible();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.tankDriveVolts(0,0);
    }
}
