package frc.robot.subsystems;

import com.revrobotics.*;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.InterpolatingTreeMap;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {

    public CANSparkMax shooterMotor1;
    public CANSparkMax shooterMotor2;

    private CANEncoder shooterEncoder;
    private CANPIDController shooterPIDController;
    
    @SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
    private InterpolatingTreeMap velocityLUT = new InterpolatingTreeMap();

    private Limelight limelight;

    private double targetRpm = 0.0;
    private double defaultRpm = 2e3;

    private boolean enabled = false;


    public Shooter(Limelight limelight) {
       this.limelight = limelight;

        shooterMotor1 = new CANSparkMax(ShooterConstants.kLeftMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(ShooterConstants.kRightMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

        shooterMotor1.setSmartCurrentLimit(60);
        shooterMotor2.setSmartCurrentLimit(60);

        shooterMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);

        shooterEncoder = shooterMotor1.getEncoder();

        shooterMotor2.follow(shooterMotor1, true);

        shooterPIDController = shooterMotor1.getPIDController();
        shooterPIDController.setFF(ShooterConstants.kSparkMaxFeedforward);
        shooterPIDController.setP(ShooterConstants.kSparkMaxP);


        populateVelocityMap();

        SmartDashboard.putNumber("Target RPM", targetRpm);
        SmartDashboard.putNumber("Current RPM", 0);
    }

    public void populateVelocityMap() {
        // TODO: implement
        velocityLUT.put(6.8, 2300.0);
        velocityLUT.put(11.1, 2100.0);
        velocityLUT.put(14.3, 2000.0);
        velocityLUT.put(22.0, 2350.0);
        velocityLUT.put(26.75, 2750.0);
        velocityLUT.put(36.0, 2950.0);
    }

    public void setVelocity(double rpmReference){
        shooterPIDController.setReference(rpmReference, ControlType.kVelocity);
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
        return limelight.targetVisible() ? velocityLUT.getInterpolated(Units.metersToFeet(limelight.getDistanceToGoal())) : defaultRpm;
    }

    public void setDynamicRpm() {
        shooterPIDController.setReference(getTargetRpm(), ControlType.kVelocity);
    }

    public boolean atSetpoint() {
        return Math.abs(getTargetRpm() - getVelocity()) < 150;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current RPM", getVelocity());
        SmartDashboard.putNumber("Predicted RPM", getTargetRpm());
    }

    public void disable() {
        enabled = false;
        shooterMotor1.set(0);
    }

    public void enable() {
        enabled = true;
    }

    public void setSetpoint(double rpm) {
        defaultRpm = rpm;
    }
}
