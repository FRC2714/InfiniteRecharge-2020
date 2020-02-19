package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class AutoIntake extends CommandBase {

    private Intake intake;
    private Conveyor conveyor;
    private IntakeType intakeType;

    public AutoIntake(Intake intake, Conveyor conveyor, IntakeType intakeType){
        this.intake = intake;
        this.conveyor = conveyor;
        this.intakeType = intakeType;
    }

    @Override
    public void initialize() {
        switch (intakeType){
            case NORMAL_INTAKE:
                intake.intakePowerCells();
                conveyor.horizontalConveyor.set(0.5);
                conveyor.setIntaking(true);
                break;
            case NORMAL_EXTAKE:
                intake.extakePowerCells();
                conveyor.horizontalConveyor.set(-0.4);
                conveyor.verticalConveyor.set(-0.4);
                conveyor.setExtaking(true);
                break;
            case FORCED_INTAKE:
                intake.intakePowerCells();
                conveyor.horizontalConveyor.set(0.7);
                conveyor.verticalConveyor.set(0.7);
                conveyor.setIntaking(true);
                break;

        }
    }

    @Override
    public void end(boolean interrupted) {
        if(intakeType.equals(IntakeType.NORMAL_INTAKE))
            conveyor.setIntaking(false);
        conveyor.setExtaking(false);
        intake.setSerializerPower(0);
        intake.setIntakePower(0);
        conveyor.moveAll(0);
    }

    public enum IntakeType{
        NORMAL_INTAKE,
        NORMAL_EXTAKE,
        FORCED_INTAKE,
        FORCED_EXTAKE
    }

}
