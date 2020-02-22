package frc.robot.commands.ballmanager;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.ToggledBoolean;

public class SingleShot extends CommandBase {

    double ballsShot;
    private Conveyor conveyor;
    private Shooter shooter;
    ToggledBoolean shotToggle;
    private Intake intake;

    public SingleShot(Conveyor conveyor, Shooter shooter, Intake intake){
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.intake = intake;
        shotToggle = new ToggledBoolean(shooter::atSetpoint);
    }

    @Override
    public void initialize() {
        ballsShot = 0;
    }

    @Override
    public void execute() {
        if(shotToggle.getToggled() && Math.abs(shooter.getVelocity() - shooter.getTargetRpm()) > 500)
            ballsShot++;

        conveyor.setIntaking(true);
        conveyor.setExtaking(true);
        intake.intakePowerCells();
        if (shooter.atSetpoint()) {
            conveyor.horizontalConveyor.set(0.35);
            conveyor.verticalConveyor.set(0.7);
        } else {
            conveyor.horizontalConveyor.set(0);
            conveyor.verticalConveyor.set(0);
        }
    }

    @Override
    public boolean isFinished() {
        return ballsShot != 0;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
