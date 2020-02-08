package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DriverControl extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier getRawX;
    private final DoubleSupplier getRawPivot;

    public DriverControl(Drivetrain subsystem, DoubleSupplier getRawX, DoubleSupplier getRawPivot) {
        drivetrain = subsystem;
        addRequirements(subsystem);
        this.getRawX = getRawX;
        this.getRawPivot = getRawPivot;
    }

    @Override
    public void execute() {

        double rawX = getRawX.getAsDouble();
        double rawPivot = getRawPivot.getAsDouble();

        if (Math.abs(getRawX.getAsDouble()) < 0.07)
            rawX = 0;

        if (Math.abs(getRawPivot.getAsDouble()) < 0.1)
            rawPivot = 0;

        if(!drivetrain.isControlsFlipped())
            drivetrain.curvatureDrive(-rawX, rawPivot);
        else
            drivetrain.curvatureDrive(rawX, rawPivot);

    }
}
