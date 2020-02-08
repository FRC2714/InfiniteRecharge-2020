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

    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;

    private CANEncoder shooterEncoder;
    private CANPIDController sparkMaxPIDController;
    
    @SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
    private InterpolatingTreeMap velocityLUT = new InterpolatingTreeMap();

    private SimpleMotorFeedforward flywheelFeedforward=
            new SimpleMotorFeedforward(ShooterConstants.kStatic,ShooterConstants.kV,ShooterConstants.kA);


    private DoubleSupplier distanceToGoal;

    public Shooter(DoubleSupplier distanceToGoal) {
        super(new PIDController(ShooterConstants.kWPILibP,0,0));
        this.distanceToGoal = distanceToGoal;

        getController().disableContinuousInput();

        SmartDashboard.putData("WPILib Shooter PID", getController());

        shooterMotor1 = new CANSparkMax(ShooterConstants.kLeftMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(ShooterConstants.kRightMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

        shooterMotor1.setSmartCurrentLimit(50);
        shooterMotor2.setSmartCurrentLimit(50);
        shooterMotor2.follow(shooterMotor1);

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
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        shooterMotor1.set(output + flywheelFeedforward.calculate(setpoint));
    }

    @Override
    protected double getMeasurement() {
        return shooterEncoder.getVelocity();
    }

    /**
     * Returns the target velocity based on current distance from goal
     * @return target velocity in RPM for shooter
     */
    public double getTargetVelocity() {
        return velocityLUT.getInterpolated(distanceToGoal.getAsDouble());
    }

    @Override
    public void periodic() {
        setSetpoint(getTargetVelocity());
        super.periodic();
    }

}
