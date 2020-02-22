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
        serializerMotor = new CANSparkMax(IntakeConstants.kSerializerMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

        intakeMotor.setSmartCurrentLimit(40);
        serializerMotor.setSmartCurrentLimit(40);

        intakeMotor.setInverted(false);
        serializerMotor.setInverted(true);
    }

    public void intakePowerCells() {
        setSerializerPower(IntakeConstants.kSerializerPower);
        setIntakePower(IntakeConstants.kIntakePower);
    }

    public void extakePowerCells() {
        setSerializerPower(-IntakeConstants.kSerializerPower);
        setIntakePower(-IntakeConstants.kIntakePower);
    }

    public void disbale() {
        setSerializerPower(0);
        setIntakePower(0);
    }

    public void setIntakePower(double power){
        intakeMotor.set(power);
    }

    public void setSerializerPower(double power){
        serializerMotor.set(power);
    }

}
