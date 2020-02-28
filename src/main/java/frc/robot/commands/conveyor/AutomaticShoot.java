package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutomaticShoot extends CommandBase {

    private Conveyor conveyor;
    private Shooter shooter;
    private double rpm;
    private boolean keepMotorRunning = false;

    private Intake intake;
    private double ballsToShoot;

    public AutomaticShoot(Shooter shooter, Conveyor conveyor, Intake intake, double rpm, boolean keepMotorRunning,
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
        shooter.setRPM(rpm);
        conveyor.enable();
        conveyor.setConveyorState(Conveyor.ConveyorState.SHOOTING);
        intake.intakePowerCells();
        shooter.setTargetRpm(rpm);
        System.out.println("Automatic Shoot Triggered");
    }

    @Override
    public boolean isFinished() {
        return shooter.getBallsShot() >= ballsToShoot;
    }

    @Override
    public void end(boolean interrupted) {
        if (!keepMotorRunning)
            shooter.setShooterPower(0);
        conveyor.setConveyorState(Conveyor.ConveyorState.DEFAULT);
        intake.disbale();
        System.out.println("Automatic Shoot Ended. Balls Shot: " + shooter.getBallsShot());
        conveyor.updatePowerCellCount(shooter.getBallsShot());
        shooter.resetBallsShot();
    }



}
