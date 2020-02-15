package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends PIDSubsystem {

    private CANSparkMax climberMotor1;
    private CANEncoder climberEncoder;

    private double targetHeightInches = 0.0;

    private SimpleMotorFeedforward climberFeedforward =
            new SimpleMotorFeedforward(ClimberConstants.kV,0,0);

    public Climber(){
        super(new PIDController(ClimberConstants.kP,0,0));
        climberMotor1 = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);

        climberMotor1.setSmartCurrentLimit(30);
        climberEncoder = climberMotor1.getEncoder();

    }

    public void setClimber(double power){
        climberMotor1.set(power);
    }

    @Override
    public double getMeasurement() {
        return (climberEncoder.getPosition() / ClimberConstants.kGearRatio)
                * 2 * Math.PI * ClimberConstants.kSprocketRadius;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        climberMotor1.set(output + climberFeedforward.calculate(setpoint));
    }
}
