package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private CANSparkMax intakeMotor1;
    private CANSparkMax intakeMotor2;

    public Intake() {
//        intakeMotor1 = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
//        intakeMotor2 = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);

        intakeMotor1.setSmartCurrentLimit(40);
        intakeMotor2.setSmartCurrentLimit(40);

        intakeMotor2.follow(intakeMotor1);
    }

    public void intakePowerCell() {
        intakeMotor1.set(0.5);
    }

    public void extakePowerCell() {
        intakeMotor1.set(-0.5);
    }

}
