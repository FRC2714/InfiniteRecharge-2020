package frc.robot.subsystems;

import com.revrobotics.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.InterpolatingTreeMap;
import frc.robot.utils.ToggledBreakBeam;

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

    private ToggledBreakBeam shooterBeam;

    private int ballsShot = 0;

    private double rpmIncrement = 0;


    public Shooter(Limelight limelight) {
       this.limelight = limelight;
       rpmIncrement = 0;

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

        shooterBeam = new ToggledBreakBeam(new DigitalInput(7));
    }

    public void populateVelocityMap() {
        // TODO: implement
        /*
        velocityLUT.put(6.8, 2300.0);
        velocityLUT.put(11.1, 2100.0);
        velocityLUT.put(14.3, 2000.0);
        velocityLUT.put(22.0, 2350.0);
        velocityLUT.put(26.75, 2750.0);
        velocityLUT.put(36.0, 2950.0);
        */

        velocityLUT.put(6.8, 2200.0);
        velocityLUT.put(11.1, 2050.0);
        velocityLUT.put(14.3, 2100.0);
        velocityLUT.put(22.0, 2350.0);
        velocityLUT.put(26.75, 2650.0);//2650.0
        velocityLUT.put(36.0, 3500.0);//2850.0
    }

    public void setRPM(double rpmReference){
        shooterPIDController.setReference(rpmReference, ControlType.kVelocity);
        setTargetRpm(rpmReference);
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

    public void incrementRPM(){
        rpmIncrement += 50;
    }

    public void decrementRPM(){
        rpmIncrement -= 50;
    }

    public void setRpmIncrement(double rpmIncrement){
        this.rpmIncrement = rpmIncrement;
    }

    /**
     * Returns the target velocity based on current distance from goal
     * @return target velocity in RPM for shooter
     */
    public double getTargetRpm() {
        return limelight.targetVisible() ? velocityLUT.getInterpolated(Units.metersToFeet(limelight.getDistanceToGoal())) + rpmIncrement: defaultRpm;
    }

    public void setDynamicRpm() {
        shooterPIDController.setReference(getTargetRpm(), ControlType.kVelocity);
        setTargetRpm(getTargetRpm());
    }

    public boolean atSetpoint() {
        return Math.abs(targetRpm - getVelocity()) < ShooterConstants.kVelocityTolerance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current RPM", getVelocity());
        SmartDashboard.putNumber("Predicted RPM", getTargetRpm());
        SmartDashboard.putBoolean("Is Shooter Reached Speed", atSetpoint());
        SmartDashboard.putBoolean("Shooter Beam", shooterBeam.getState());
        SmartDashboard.putNumber("RPM Increment = ", rpmIncrement);

        shooterBeam.update();
        if (shooterBeam.getToggled()) {
            ballsShot++;
            System.out.println("Ball was Shot -- " + ballsShot);
        }
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

    public void resetBallsShot() {
        ballsShot = 0;
    }

    public int getBallsShot() {
        return ballsShot;
    }
}
