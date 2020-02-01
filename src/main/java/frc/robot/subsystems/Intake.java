package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private CANSparkMax intakeMotor1;
    private CANSparkMax serializerMotor1;
    private CANSparkMax serializerMotor2;

    public Intake() {
//        intakeMotor1 = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
//        serializerMotor1 = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);
//        serializerMotor2 = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);


        intakeMotor1.setSmartCurrentLimit(40);
    }

    public void intakePowerCell() {
        intakeMotor1.set(0.5);
    }

    public void extakePowerCell() {
        intakeMotor1.set(-0.5);
    }

}
