package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    CANSparkMax climberMotor1;
    CANSparkMax climberMotor2;


    public Climber(){
        climberMotor1 = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
        climberMotor2 = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);

        climberMotor2.follow(climberMotor1);
        climberMotor1.setSmartCurrentLimit(30);

        climberMotor2.setInverted(false);

    }

    public void setClimber(double power){
        climberMotor1.set(power);
    }

}
