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
        setSerializerPower(0.5);
        setIntakePower(0.5);
    }

    public void extakePowerCells() {
        setIntakePower(-0.5);
        setSerializerPower(-0.5);
    }

    public void setIntakePower(double power){
        intakeMotor.set(power);
    }

    public void setSerializerPower(double power){
        serializerMotor.set(power);
    }

}
