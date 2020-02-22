package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class AutomaticShooter extends CommandBase {

    private Conveyor conveyor;
    private Shooter shooter;
    private double rpm;

    public AutomaticShooter(Shooter shooter, Conveyor conveyor, double rpm){
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
        shooter.setVelocity(rpm);
        conveyor.enable();
        conveyor.setConveyorState(Conveyor.ConveyorState.SHOOTING);
    }

    @Override
    public void execute() {

    }


    @Override
    public void end(boolean interrupted) {
        shooter.setShooterPower(0);
        conveyor.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
