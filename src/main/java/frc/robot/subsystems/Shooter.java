package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.utils.InterpolatingTreeMap;

public class Shooter extends PIDSubsystem {

    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;

    private CANEncoder shooterEncoder;
    
    @SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
    private InterpolatingTreeMap velocityLUT = new InterpolatingTreeMap();

    private SimpleMotorFeedforward flywheelFeedforward=
            new SimpleMotorFeedforward(0,0,0);

    private Limelight limelight;

    public Shooter(Limelight limelight) {
        super(new PIDController(0,0,0));
        this.limelight = limelight;

        SmartDashboard.putData("Shooter PID Controller", getController());

        shooterMotor1 = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);

        shooterMotor1.setSmartCurrentLimit(50);
        shooterMotor2.setSmartCurrentLimit(50);
        shooterMotor2.follow(shooterMotor1);
        shooterEncoder = shooterMotor1.getEncoder();

        populateVelocityMap();
    }

    public void populateVelocityMap() {
        // TODO: implement
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
        return velocityLUT.getInterpolated(limelight.getDistanceToGoal());
    }

    @Override
    public void periodic() {
        setSetpoint(getTargetVelocity());
        super.periodic();
    }

}
