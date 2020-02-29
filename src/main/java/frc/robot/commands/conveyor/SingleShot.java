package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SingleShot extends CommandBase {

    private Shooter shooter;
    private Conveyor conveyor;
    private Intake intake;

    public SingleShot(Shooter shooter, Intake intake, Conveyor conveyor){
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        shooter.resetBallsShot();
        conveyor.enable();
        conveyor.setConveyorState(Conveyor.ConveyorState.SHOOTING);
        intake.setIntakePower(0.5);
        System.out.println("Single Shot Initialized. Balls Shot = " + shooter.getBallsShot());
    }

    @Override
    public void execute() {
        if(shooter.getBallsShot() != 0) {
            conveyor.disable();
            conveyor.setConveyorState(Conveyor.ConveyorState.DEFAULT);
            conveyor.updatePowerCellCount(shooter.getBallsShot());
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.disbale();
        shooter.disable();
        conveyor.disable();
        System.out.println("Single Shot Ended. Balls Shot: " + shooter.getBallsShot());
        shooter.resetBallsShot();
        conveyor.setConveyorState(Conveyor.ConveyorState.DEFAULT);
    }
}
