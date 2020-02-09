package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class ManualShooter extends CommandBase {

    private Shooter shooter;
    private Conveyor conveyor;
    private double rpm;

    public ManualShooter(Shooter shooter, Conveyor conveyor, double rpm){
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
        conveyor.setConveyorState(Conveyor.ConveyorState.SHOOTING);
        shooter.setSparkMaxVelocity(rpm);
        shooter.setSetpoint(rpm);
        shooter.enable();
    }

    @Override
    public void execute() {
        conveyor.countBallsShot();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disable();
        switch (conveyor.getPowerCellsStored()){
            case 0:
                conveyor.setConveyorState(Conveyor.ConveyorState.EMPTY);
                break;
            case 1:
                conveyor.setConveyorState(Conveyor.ConveyorState.ONE);
                break;
            case 2:
                conveyor.setConveyorState(Conveyor.ConveyorState.TWO);
                break;
            case 3:
                conveyor.setConveyorState(Conveyor.ConveyorState.THREE);
                break;
            case 4:
                conveyor.setConveyorState(Conveyor.ConveyorState.FOUR);
                break;
            case 5:
                conveyor.setConveyorState(Conveyor.ConveyorState.FIVE);
                break;
            default:
                conveyor.setConveyorState(Conveyor.ConveyorState.ERROR);
                break;
        }
    }

}
