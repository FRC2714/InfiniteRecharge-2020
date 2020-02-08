package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private CANSparkMax intakeMotor;
    private CANSparkMax intakeWristMotor;
    private CANSparkMax serializerMotor;


    public Intake() {
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        intakeWristMotor = new CANSparkMax(IntakeConstants.kIntakeWristMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        serializerMotor = new CANSparkMax(IntakeConstants.kSerializerMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

        intakeMotor.setSmartCurrentLimit(40);
        intakeWristMotor.setSmartCurrentLimit(40);

        serializerMotor.setSmartCurrentLimit(40);
    }

    public void intakePowerCells() {
        intakeMotor.set(0.5);
    }

    public void extakePowerCells() { intakeMotor.set(-0.5); }


}
