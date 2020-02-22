package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutomaticShooter extends CommandBase {

    private Conveyor conveyor;
    private Shooter shooter;
    private double rpm;
    private boolean keepMotorRunning = false;

    private Intake intake;
    private double ballsToShoot;

    public AutomaticShooter(Shooter shooter, Conveyor conveyor, Intake intake, double rpm, boolean keepMotorRunning,
                            double ballsToShoot){
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.rpm = rpm;
        this.keepMotorRunning = keepMotorRunning;
        this.intake = intake;
        this.ballsToShoot = ballsToShoot;
    }

    @Override
    public void initialize() {
        shooter.resetBallsShot();
        shooter.setVelocity(rpm);
        conveyor.enable();
        conveyor.setConveyorState(Conveyor.ConveyorState.SHOOTING);
        intake.intakePowerCells();
    }

    @Override
    public void execute() {

    }


    @Override
    public void end(boolean interrupted) {
        if (!keepMotorRunning)
            shooter.setShooterPower(0);
        conveyor.disable();
        intake.disbale();
        shooter.resetBallsShot();
    }

    @Override
    public boolean isFinished() {
        return shooter.getBallsShot() >= ballsToShoot;
    }
}
