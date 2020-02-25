package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoIntake extends CommandBase {

    private Shooter shooter;
    private Intake intake;
    private Conveyor conveyor;
    private IntakeType intakeType;

    public AutoIntake(Shooter shooter, Intake intake, Conveyor conveyor, IntakeType intakeType){
        addRequirements(shooter,intake,conveyor);
        this.shooter = shooter;
        this.intake = intake;
        this.conveyor = conveyor;
        this.intakeType = intakeType;
    }

    @Override
    public void initialize() {
        conveyor.enable();
        switch (intakeType){
            case INTAKE:
                conveyor.setConveyorState(Conveyor.ConveyorState.DEFAULT);
                intake.intakePowerCells();
                break;
            case EXTAKE:
                intake.extakePowerCells();
                conveyor.setConveyorState(Conveyor.ConveyorState.EXTAKING);
                break;
            case SHOOT:
                conveyor.setConveyorState(Conveyor.ConveyorState.SHOOTING);
        }
    }

    @Override
    public void execute() {
        // logic handled in conveyor periodic
    }


    @Override
    public void end(boolean interrupted) {
        intake.disbale();
        conveyor.setConveyorState(Conveyor.ConveyorState.DEFAULT);
    }

    public enum IntakeType{
        INTAKE,
        EXTAKE,
        SHOOT
    }

}
