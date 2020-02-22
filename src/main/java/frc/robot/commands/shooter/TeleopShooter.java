package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class TeleopShooter extends CommandBase {

    private Shooter shooter;
    private Conveyor conveyor;
    private double rpm;

    public TeleopShooter(Shooter shooter, Conveyor conveyor, double rpm){
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
        shooter.setVelocity(rpm);
        conveyor.setConveyorState(Conveyor.ConveyorState.SHOOTING);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Current Output 1", shooter.shooterMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Current Output 2", shooter.shooterMotor2.getOutputCurrent());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterPower(0);
        SmartDashboard.putNumber("Current Output 1", 0);
        SmartDashboard.putNumber("Current Output 2", 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rpm - shooter.getVelocity()) < (.05 *rpm);
    }


}
