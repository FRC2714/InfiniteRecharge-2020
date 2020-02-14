package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends CommandBase {

    private Shooter shooter;
    private Conveyor conveyor;
    private double rpm;

    public AutoShooter(Shooter shooter, Conveyor conveyor, double rpm){
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
        conveyor.setConveyorState(Conveyor.ConveyorState.SHOOTING);
        shooter.setSparkMaxVelocity(rpm);
        shooter.setSetpoint(rpm);
        shooter.enable();
    }

    @Override
    public void execute() {
        conveyor.countPowerCellsShot();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disable();
        conveyor.updateEnum();
    }

}
