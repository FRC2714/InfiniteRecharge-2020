package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class AutomatedIntake extends CommandBase {

    private Conveyor conveyor;

    public AutomatedIntake(Conveyor conveyor){
        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {
        conveyor.setIntakeState(true);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setIntakeState(false);
    }
}
