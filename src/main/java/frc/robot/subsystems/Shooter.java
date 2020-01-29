package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Shooter extends PIDSubsystem {

    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;

    private CANEncoder shooterEncoder;
    

    SimpleMotorFeedforward flywheelFeedforward=
            new SimpleMotorFeedforward(0,0,0);

    public Shooter() {
        super(new PIDController(0,0,0));

//      intakeMotor1 = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
//      intakeMotor2 = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);

        shooterMotor1.setSmartCurrentLimit(50);
        shooterMotor2.setSmartCurrentLimit(50);
        shooterMotor2.follow(shooterMotor1);
        shooterEncoder = shooterMotor1.getEncoder();
    }

    public void setShooterPower(double power){
        shooterMotor1.set(power);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        shooterMotor1.set(output + flywheelFeedforward.calculate(setpoint));
    }

    @Override
    protected double getMeasurement() {
        return shooterEncoder.getVelocity();
    }

}
