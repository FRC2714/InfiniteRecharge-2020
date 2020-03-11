package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class MoveClimber extends CommandBase {

    private Climber climber;
    private ClimberMotionType climberMotionType;

    public MoveClimber(Climber climber, ClimberMotionType climberMotionType){
        addRequirements(climber);
        this.climber = climber;
        this.climberMotionType = climberMotionType;
    }

    @Override
    public void initialize() {
        System.out.println("Climber Init");
    }

    @Override
    public void execute() {
        switch (climberMotionType){
            case AUTO_EXTEND:
                climber.setToTargetInches(10);
                break;
            case AUTO_LOCK:
                break;
            case MANUAL_EXTEND:
                climber.setClimberUp();
                break;
            case MANUAL_RETRACT:
                climber.setClimberDown();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.setPower(0);
        System.out.println("Climber End : " + climber.getPosition());

    }

    public enum ClimberMotionType {
        MANUAL_EXTEND,
        MANUAL_RETRACT,
        AUTO_EXTEND,
        AUTO_LOCK,
    }

}
