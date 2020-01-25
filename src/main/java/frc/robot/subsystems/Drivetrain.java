package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drivetrain.DriverControl;

public class Drivetrain extends SubsystemBase {

    // motors
    private CANSparkMax lMotor0;
    private CANSparkMax lMotor1;
    private CANSparkMax lMotor2;
    private CANSparkMax rMotor0;
    private CANSparkMax rMotor1;
    private CANSparkMax rMotor2;

    // neo encoder
    private CANEncoder leftNeoEncoder;
    private CANEncoder rightNeoEncoder;

    // Spark Max Pid

    private CANPIDController leftPIDController;
    private CANPIDController rightPIDController;

    // Encoders
    private final Encoder leftEncoder = new Encoder(
            DriveConstants.kLeftEncoderPorts[0],
            DriveConstants.kLeftEncoderPorts[1],
            true,
            CounterBase.EncodingType.k4X
    );

    private final Encoder rightEncoder = new Encoder(
            DriveConstants.kRightEncoderPorts[0],
            DriveConstants.kRightEncoderPorts[1],
            true,
            CounterBase.EncodingType.k4X
    );

    // Gyro
    private AHRS navx;
    private ADIS16470_IMU adisIMU = new ADIS16470_IMU();


    private DifferentialDrive drive;

    private DifferentialDriveOdometry odometry;

    // Current pose
    private Pose2d currentPose = new Pose2d();

    private DifferentialDriveKinematics kinematics
            = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);


    private NetworkTable live_dashboard = NetworkTableInstance.getDefault().getTable("Live_Dashboard");

    public Drivetrain() {
        lMotor0 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        lMotor1 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        lMotor2 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        rMotor0 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
        rMotor1 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
        rMotor2 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

        lMotor1.follow(lMotor0);
        rMotor1.follow(rMotor0);
        lMotor2.follow(lMotor0);
        rMotor2.follow(rMotor0);

        lMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
        lMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        lMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        rMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        lMotor0.enableVoltageCompensation(12.0);
        rMotor0.enableVoltageCompensation(12.0);

        lMotor0.setSmartCurrentLimit(50);
        lMotor1.setSmartCurrentLimit(50);
        lMotor2.setSmartCurrentLimit(50);

        rMotor0.setSmartCurrentLimit(50);
        rMotor1.setSmartCurrentLimit(50);
        rMotor2.setSmartCurrentLimit(50);

        drive = new DifferentialDrive(lMotor0, rMotor0);
        drive.setSafetyEnabled(false);

        leftEncoder.reset();
        rightEncoder.reset();

        leftEncoder.setDistancePerPulse(2 * Math.PI * DriveConstants.kWheelRadius / DriveConstants.kShaftEncoderResolution);
        rightEncoder.setDistancePerPulse(2 * Math.PI * DriveConstants.kWheelRadius / DriveConstants.kShaftEncoderResolution);

        leftNeoEncoder = lMotor0.getEncoder();
        rightNeoEncoder = rMotor0.getEncoder();


        leftNeoEncoder.setPosition(0);
        rightNeoEncoder.setPosition(0);
        navx = new AHRS(SPI.Port.kMXP);
        navx.reset();
        navx.zeroYaw();

        drive = new DifferentialDrive(lMotor0, rMotor0);
        drive.setSafetyEnabled(false);

        // TODO: set encoder distance per pulse
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        leftPIDController = lMotor0.getPIDController();
        rightPIDController = rMotor0.getPIDController();

        adisIMU.calibrate();
    }

    @Override
    public void periodic() {
        double leftDist = leftEncoder.getDistance();
        double rightDist = rightEncoder.getDistance();

        // Update the odometry in the periodic block
        odometry.update(Rotation2d.fromDegrees(getHeading()), leftDist,
                rightDist);

        SmartDashboard.putNumber("Left Encoder Meters", leftDist);
        SmartDashboard.putNumber("Right Encoder Meters", rightDist);

        live_dashboard.getEntry("robotX").setDouble(Units.metersToFeet(getPose().getTranslation().getX()));
        live_dashboard.getEntry("robotY").setDouble(Units.metersToFeet(getPose().getTranslation().getY()));
        live_dashboard.getEntry("robotHeading").setDouble(getPose().getRotation().getRadians());

        SmartDashboard.putNumber("robotX", Units.metersToFeet(getPose().getTranslation().getX()));
        SmartDashboard.putNumber("robotY", Units.metersToFeet(getPose().getTranslation().getY()));
        SmartDashboard.putNumber("robotHeading", getPose().getRotation().getDegrees());

    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
        adisIMU.reset();
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        System.out.println("FORWARD = " + fwd + " ROTATION" + rot);
        drive.arcadeDrive(fwd, rot);
    }


    public void curvatureDrive(double fwd, double rot) {
        drive.curvatureDrive(fwd, rot, true);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        lMotor0.set(leftVolts / RobotController.getBatteryVoltage());
        rMotor0.set(-rightVolts / RobotController.getBatteryVoltage());
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }


    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        navx.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(adisIMU.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }


    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return adisIMU.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public void resetAll() {
        resetOdometry(new Pose2d());
        adisIMU.reset();
        navx.reset();
    }

    public void initDefaultCommand(Joystick joystick) {
        setDefaultCommand(new DriverControl(
                this,
                () -> joystick.getRawAxis(1),
                () -> joystick.getRawAxis(4)
        ));
    }

}
