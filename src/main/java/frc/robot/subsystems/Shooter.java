package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.InterpolatingTreeMap;

import java.util.function.DoubleSupplier;

public class Shooter extends PIDSubsystem {

    public CANSparkMax shooterMotor1;
    public CANSparkMax shooterMotor2;

    private CANEncoder shooterEncoder;
    private CANPIDController sparkMaxPIDController;
    
    @SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
    private InterpolatingTreeMap velocityLUT = new InterpolatingTreeMap();

    private SimpleMotorFeedforward flywheelFeedforward=
            new SimpleMotorFeedforward(ShooterConstants.kStatic,ShooterConstants.kV,ShooterConstants.kA);


    private Limelight limelight;

    private double targetRpm = 0.0;
    private double defaultRpm = 2e3;


    public Shooter(Limelight limelight) {
       super(new PIDController(ShooterConstants.kWPILibP,0,0));
       this.limelight = limelight;

        getController().disableContinuousInput();
        getController().setTolerance(ShooterConstants.kVelocityTolerance);

        SmartDashboard.putData("WPILib Shooter PID", getController());

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

        sparkMaxPIDController = shooterMotor1.getPIDController();
        sparkMaxPIDController.setFF(ShooterConstants.kSparkMaxFeedforward);
        sparkMaxPIDController.setP(ShooterConstants.kSparkMaxP);

        populateVelocityMap();
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
        shooterMotor2.set(-power);
    }

    public void setTargetRpm(double targetRpm) {
        this.targetRpm = targetRpm;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
//        shooterMotor1.set(output + flywheelFeedforward.calculate(setpoint));
    }

    @Override
    protected double getMeasurement() {
        return shooterEncoder.getVelocity();
    }

    public double getVelocity() {
        return shooterEncoder.getVelocity();
    }

    /**
     * Returns the target velocity based on current distance from goal
     * @return target velocity in RPM for shooter
     */
    public double getTargetLimelightVelocity() {
        return limelight.targetVisible() ? velocityLUT.getInterpolated(limelight.getDistanceToGoal()) : defaultRpm;
    }

    @Override
    public void periodic() {
//        targetRpm = getTargetLimelightVelocity();
//        setSetpoint(targetRpm);
        super.periodic();
    }

}
