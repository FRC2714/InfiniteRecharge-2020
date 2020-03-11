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

    private CANSparkMax climberMotor1, climberMotor2;
    private CANPIDController climberPIDController;
    private CANEncoder climberEncoder;

    private double targetHeightInches = 0.0;

    public Climber(){
        climberMotor1 = new CANSparkMax(ClimberConstants.kLeftMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        climberMotor2 = new CANSparkMax(ClimberConstants.kRightMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        climberEncoder = climberMotor1.getEncoder();

        climberMotor1.setSmartCurrentLimit(30);

        climberMotor2.follow(climberMotor1, true);

//        climberMotor1.setInverted(false);
//        climberEncoder.setInverted(false);

        climberPIDController = climberMotor1.getPIDController();
        climberPIDController.setP(ClimberConstants.kP);

        climberMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        climberMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        climberEncoder.setPosition(0);
    }

    public void setPower(double power){
        climberMotor1.set(power);
    }

    public void setClimberDown(){
       if(climberEncoder.getPosition() > ClimberConstants.kMinHeightTicks){
            setPower(-0.75);
        } else setPower(0);
    }

    public void setClimberUp(){
        if(climberEncoder.getPosition() <= ClimberConstants.kMaxHeightTicks){
            setPower(1);
        } else setPower(0);
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


    @Override
    public void periodic() {

    }

    public double getPosition() {
        return climberEncoder.getPosition();
    }
}
