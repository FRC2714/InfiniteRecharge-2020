package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SingleShot extends CommandBase {

    private Shooter shooter;
    private Conveyor conveyor;
    private Intake intake;

    public SingleShot(Shooter shooter, Conveyor conveyor, Intake intake){
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        shooter.resetBallsShot();
        conveyor.enable();
        conveyor.setConveyorState(Conveyor.ConveyorState.SHOOTING);
        intake.intakePowerCells();
    }

    @Override
    public void execute() {
        if(shooter.getBallsShot() != 0)
            conveyor.disable();
    }
    
}
