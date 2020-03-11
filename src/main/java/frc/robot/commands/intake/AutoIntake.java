package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
        conveyor.enable();
        switch (intakeType){
            case INTAKE:
                conveyor.setConveyorState(Conveyor.ConveyorState.INTAKING);
                intake.intakePowerCells();
                break;
            case EXTAKE:
                intake.extakePowerCells();
                conveyor.setConveyorState(Conveyor.ConveyorState.EXTAKING);
                break;
            case SHOOT:
                conveyor.setConveyorState(Conveyor.ConveyorState.SHOOTING);
                break;
            case FORCED_CONVEYOR_INTAKE:
                conveyor.setConveyorState(Conveyor.ConveyorState.FORCED_CONVEYOR_INTAKE);
                break;
            case FORCED_CONVEYOR_EXTAKE:
                conveyor.setConveyorState(Conveyor.ConveyorState.FORCED_CONVEYOR_EXTAKE);
                break;
            case UNJAM_STUCK_BALL:
                intake.setSerializerPower(-Constants.IntakeConstants.kSerializerPower);
                conveyor.setConveyorState(Conveyor.ConveyorState.EXTAKING);
            case UNRESTRICTED_SHOOT:
                conveyor.setConveyorState(Conveyor.ConveyorState.UNRESTRICTED_SHOOTING);
                break;
        }

        System.out.println("Auto Intake Triggered. Type -- " + intakeType);
    }

    @Override
    public void execute() {
        // logic handled in conveyor periodic
        if(intakeType.equals(IntakeType.SHOOT))
            conveyor.setConveyorState(Conveyor.ConveyorState.SHOOTING);
    }

    @Override
    public void end(boolean interrupted) {
        intake.disbale();
        conveyor.disable();
        conveyor.setConveyorState(Conveyor.ConveyorState.DEFAULT);
        System.out.println("Auto Intake Ended. Type -- " + intakeType);
    }

    public enum IntakeType{
        INTAKE,
        EXTAKE,
        SHOOT,
        UNRESTRICTED_SHOOT,
        FORCED_CONVEYOR_INTAKE,
        FORCED_CONVEYOR_EXTAKE,
        UNJAM_STUCK_BALL
    }

}
