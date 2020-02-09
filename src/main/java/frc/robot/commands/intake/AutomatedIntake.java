package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class AutomatedIntake extends CommandBase {

    private Intake intake;
    private Conveyor conveyor;
    private IntakeType intakeType;

    public AutomatedIntake(Intake intake, Conveyor conveyor, IntakeType intakeType){
        this.intake = intake;
        this.conveyor = conveyor;
        this.intakeType = intakeType;
    }

    @Override
    public void initialize() {
        switch (intakeType){
            case NORMAL_INTAKE:
                intake.intakePowerCells();
                break;
            case NORMAL_EXTAKE:
                intake.extakePowerCells();
                break;
        }

        conveyor.setIntaking(true);
    }

    @Override
    public void end(boolean interrupted) {
        intake.extakePowerCells();
        conveyor.setIntaking(false);
    }

    enum IntakeType{
        NORMAL_INTAKE,
        NORMAL_EXTAKE,
        FORCED_INTAKE,
        FORCED_EXTAKE
    }

}
