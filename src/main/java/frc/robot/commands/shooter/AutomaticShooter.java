package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutomaticShooter extends CommandBase {

    private Conveyor conveyor;
    private Shooter shooter;
    private double rpm;
    private boolean keepMotorRunning = false;

    private Intake intake;

    public AutomaticShooter(Shooter shooter, Conveyor conveyor, Intake intake, double rpm, boolean keepMotorRunning){
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.rpm = rpm;
        this.keepMotorRunning = keepMotorRunning;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        shooter.setVelocity(rpm);
        conveyor.enable();
        conveyor.setConveyorState(Conveyor.ConveyorState.SHOOTING);
        intake.intakePowerCells();
    }

    @Override
    public void execute() {

    }


    @Override
    public void end(boolean interrupted) {
        if (!keepMotorRunning)
            shooter.setShooterPower(0);
        conveyor.disable();
        intake.disbale();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
