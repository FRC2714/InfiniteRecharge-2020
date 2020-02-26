package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

    private CANSparkMax climberMotor1;
    private CANPIDController climberPIDController;
    private CANEncoder climberEncoder;
    private Servo servo;

    private double targetHeightInches = 0.0;

    public Climber(){
        climberMotor1 = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
        climberEncoder = climberMotor1.getEncoder();
        servo = new Servo(2);

        climberMotor1.setSmartCurrentLimit(30);

//        climberMotor1.setInverted(false);
//        climberEncoder.setInverted(false);

        climberPIDController = climberMotor1.getPIDController();
        climberPIDController.setP(ClimberConstants.kP);

    }

    public void setPower(double power){
        climberMotor1.set(power);
    }

    public void setClimberDown(){
        if(climberEncoder.getPosition() > 300){
            setPower(-0.6);
        }
    }

    public void setClimberUp(){
        if(climberEncoder.getPosition() <= ClimberConstants.kMaxHeightTicks){
            setPower(0.6);
        }
    }

    public void setPIDTargetHeight(double targetHeightInches) {
        this.targetHeightInches = targetHeightInches;
        climberPIDController.setReference(targetHeightInches / (2 * Math.PI *
                ClimberConstants.kSprocketRadius), ControlType.kPosition);
    }

    public boolean atSetpoint() {
        return Math.abs(targetHeightInches - (climberEncoder.getPosition() * 2 * Math.PI * ClimberConstants.kSprocketRadius))
                    < ClimberConstants.kToleranceInches;
    }

    public double getClimberHeightInches(){
        return Math.abs(targetHeightInches - (climberEncoder.getPosition() * 2 * Math.PI * ClimberConstants.kSprocketRadius));
    }

    public void setToTargetTicks(double targetTicks){
        if(Math.abs(climberEncoder.getPosition() - targetTicks) < 200){
            climberMotor1.set(0);
        } else if(climberEncoder.getPosition() < targetTicks)
            climberMotor1.set(0.6);
        else
            climberMotor1.set(-0.6);
    }

    public void setToTargetInches(double targetHeightInches){
        this.targetHeightInches = targetHeightInches;
        if(atSetpoint())
            climberMotor1.set(0);
        else if(getClimberHeightInches() < targetHeightInches)
            climberMotor1.set(0.6);
        else
            climberMotor1.set(-0.6);
    }

    public void disable() {
        climberMotor1.set(0);
    }

    public void setClimberLock(boolean servoLock){
        if(servoLock)
            servo.set(ClimberConstants.servoLockPosition);
        else
            servo.set(ClimberConstants.servoUnlockPosition);
    }

    @Override
    public void periodic() {

    }

}
