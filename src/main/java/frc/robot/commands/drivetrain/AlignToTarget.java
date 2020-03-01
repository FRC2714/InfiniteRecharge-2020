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
import java.util.function.DoubleSupplier;

public class AlignToTarget extends ProfiledPIDCommand {
    private Limelight limelight;
    private Drivetrain drivetrain;
    private boolean isAutoEnabled = false;

    public AlignToTarget(Drivetrain drivetrain, Limelight limelight) {
        this(limelight, drivetrain, () -> 0);
    }

    public AlignToTarget(Drivetrain drivetrain, Limelight limelight, boolean isAutoEnabled) {
        this(limelight, drivetrain, () -> 0);
        this.isAutoEnabled = isAutoEnabled;
    }

    public AlignToTarget(Limelight limelight, Drivetrain drive, DoubleSupplier rawY) {
        super(
                new ProfiledPIDController(DriveConstants.kAlignP, 0, DriveConstants.kAlignD,
                        new TrapezoidProfile.Constraints(100, 300)),
                // Close loop on heading
                limelight::getXAngleOffset,
                // set reference to target
                0,
                // pipe to turn robot
                (output, setpoint) -> drive.arcadeDrive(-rawY.getAsDouble(), Math.signum(output) * 0.16 + output),
                // require drive
                drive
        );
        addRequirements(drive);

        this.limelight = limelight;
        this.drivetrain = drive;
        getController().enableContinuousInput(-180, 180);

        // TODO: set tolerances
        getController().setTolerance(.75, 4);
    }



    @Override
    public void initialize() {
        limelight.setLED(true);
    }

    @Override
    public boolean isFinished() {
        if(isAutoEnabled)
            return
                getController().atGoal() || !limelight.targetVisible();
        else
            return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveVolts(0, 0);

        if (!isAutoEnabled)
            limelight.setLED(false);
    }
}
