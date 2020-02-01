package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ManualShooter extends CommandBase {

    private Conveyor conveyor;

    public ManualShooter(Conveyor conveyor){
        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {
        conveyor.setConveyorState(Conveyor.ConveyorState.SHOOTING);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setConveyorState(Conveyor.ConveyorState.EMPTY);
    }
}
