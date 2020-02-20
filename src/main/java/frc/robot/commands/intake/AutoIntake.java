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
        this.shooter = shooter;
        this.intake = intake;
        this.conveyor = conveyor;
        this.intakeType = intakeType;
    }

    @Override
    public void initialize() {
        switch (intakeType){
            case NORMAL_INTAKE:
                intake.intakePowerCells();
                conveyor.setIntaking(true);
                break;
            case FORCED_INTAKE:
                intake.intakePowerCells();
                conveyor.setExtaking(true);
                conveyor.horizontalConveyor.set(0.2);
                conveyor.verticalConveyor.set(0);
                break;
            case NORMAL_EXTAKE:
                intake.extakePowerCells();
                conveyor.setExtaking(true);
                conveyor.horizontalConveyor.set(-0.4);
                conveyor.verticalConveyor.set(-0.4);
                break;
            case FORCED_SHOOT:
                conveyor.setIntaking(true);
                conveyor.setExtaking(true);
                intake.intakePowerCells();
                if(shooter.atSetpoint()) {
                    conveyor.horizontalConveyor.set(0.35);
                    conveyor.verticalConveyor.set(0.7);
                }
                break;

        }
    }

    @Override
    public void execute() {
        switch (intakeType){
            case FORCED_SHOOT:
                conveyor.setIntaking(true);
                conveyor.setExtaking(true);
                intake.intakePowerCells();
                if(shooter.atSetpoint()) {
                    conveyor.horizontalConveyor.set(0.35);
                    conveyor.verticalConveyor.set(0.7);
                } else {
                    conveyor.horizontalConveyor.set(0);
                    conveyor.verticalConveyor.set(0);
                }
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
        FORCED_SHOOT,
        FORCED_INTAKE
    }

}
