package frc.robot.subsystems;

import com.revrobotics.*;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.InterpolatingTreeMap;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {

    public CANSparkMax shooterMotor1;
    public CANSparkMax shooterMotor2;

    private CANEncoder shooterEncoder;
    private CANPIDController sparkMaxPIDController;
    
    @SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
    private InterpolatingTreeMap velocityLUT = new InterpolatingTreeMap();

    private Limelight limelight;

    private double targetRpm = 0.0;
    private double defaultRpm = 2e3;

    private boolean enabled = false;


    public Shooter(Limelight limelight) {
       this.limelight = limelight;

        shooterMotor1 = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);

        shooterMotor1.setSmartCurrentLimit(60);
        shooterMotor2.setSmartCurrentLimit(60);

//        shooterMotor1.setInverted(false);
//        shooterMotor2.setInverted(false);
//
//        shooterMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
//        shooterMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        shooterEncoder = shooterMotor1.getEncoder();

        shooterMotor2.follow(shooterMotor1, true);

        sparkMaxPIDController = shooterMotor1.getPIDController();
        sparkMaxPIDController.setFF(ShooterConstants.kSparkMaxFeedforward);
        sparkMaxPIDController.setP(ShooterConstants.kSparkMaxP);


        populateVelocityMap();

        SmartDashboard.putNumber("Target RPM", targetRpm);
        SmartDashboard.putNumber("Current RPM", 0);

    }

    public void populateVelocityMap() {
        // TODO: implement
    }

    public void setSparkMaxVelocity(double rpmReference){
        sparkMaxPIDController.setReference(rpmReference, ControlType.kVelocity);
    }

    public void setSparkMaxSmartVelocity(double rpmReference){
        sparkMaxPIDController.setReference(rpmReference, ControlType.kSmartVelocity);
    }

    public void setShooterPower(double power){
        shooterMotor1.set(power);
    }

    public void setTargetRpm(double targetRpm) {
        this.targetRpm = targetRpm;
    }


    public double getVelocity() { // in rpm
        return shooterEncoder.getVelocity();
    }

    /**
     * Returns the target velocity based on current distance from goal
     * @return target velocity in RPM for shooter
     */
    public double getTargetRpm() {
        return limelight.targetVisible() ? velocityLUT.getInterpolated(limelight.getDistanceToGoal()) : defaultRpm;
    }

    public boolean atSetpoint() {
        return Math.abs(targetRpm - getVelocity()) < targetRpm * .05;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current RPM", getVelocity());
        double newTargetRpm = SmartDashboard.getNumber("Target RPM", 0);

        if (enabled) {
            if (targetRpm != newTargetRpm) {
                targetRpm = newTargetRpm;
                setSparkMaxVelocity(targetRpm);
            }
        }
    }

    public void disable() {
        enabled = false;
        shooterMotor1.set(0);
    }

    public void enable() {
        enabled = true;
    }

}
