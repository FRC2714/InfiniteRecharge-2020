package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class AutomaticShooter extends CommandBase {

    private Conveyor conveyor;
    private Shooter shooter;
    private double rpm;

    public AutomaticShooter(Conveyor conveyor, Shooter shooter, double rpm){
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
        shooter.setSparkMaxVelocity(rpm);
        shooter.setTargetRpm(rpm);
    }

    @Override
    public void execute() {
        
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
