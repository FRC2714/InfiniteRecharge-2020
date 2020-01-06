package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriverControl extends CommandBase {
    private final Drivetrain drivetrain;

    public DriverControl(Drivetrain subsystem) {
        drivetrain = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double rawX = RobotContainer.driverStick.getRawAxis(1);
        double rawPivot = RobotContainer.driverStick.getRawAxis(4);

        if(Math.abs(rawX) < 0.07)
            rawX = 0;

        if(Math.abs(rawPivot) < 0.1)
            rawPivot = 0;

        drivetrain.arcadeDrive(-rawX, rawPivot);
    }
}
