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
            false,
            CounterBase.EncodingType.k4X
    );

    private final Encoder rightEncoder = new Encoder(
            DriveConstants.kRightEncoderPorts[0],
            DriveConstants.kRightEncoderPorts[1],
            true,
            CounterBase.EncodingType.k4X
    );

    // Gyro
    private AHRS navx = new AHRS(SPI.Port.kMXP);


    private DifferentialDrive drive;

    private DifferentialDriveOdometry externalOdometry;
    private DifferentialDriveOdometry internalOdometry;

    private boolean isControlsFlipped = false;

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
        lMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        lMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);

        rMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);

        lMotor0.enableVoltageCompensation(12.0);
        rMotor0.enableVoltageCompensation(12.0);

        lMotor0.setSmartCurrentLimit(60);
        lMotor1.setSmartCurrentLimit(60);
        lMotor2.setSmartCurrentLimit(60);

        rMotor0.setSmartCurrentLimit(60);
        rMotor1.setSmartCurrentLimit(60);
        rMotor2.setSmartCurrentLimit(60);

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

        drive = new DifferentialDrive(lMotor0, rMotor0);
        drive.setSafetyEnabled(false);

        // TODO: set encoder distance per pulse
        externalOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        internalOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        leftPIDController = lMotor0.getPIDController();
        rightPIDController = rMotor0.getPIDController();

        resetAll();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return externalOdometry.getPoseMeters();
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
        navx.zeroYaw();
        internalOdometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
        externalOdometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
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
        leftNeoEncoder.setPosition(0);
        rightNeoEncoder.setPosition(0);
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
        navx.zeroYaw();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(navx.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * @return True if external encoders and internal encoders conflict
     */
    public boolean isEncoderError() {
        return internalOdometry.getPoseMeters().getTranslation().getDistance(externalOdometry.getPoseMeters().getTranslation()) > 0.5;
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public DifferentialDriveKinematics getKinematics() {
        return DriveConstants.kDriveKinematics;
    }

    public void resetAll() {
        resetOdometry(new Pose2d());
        navx.reset();
    }

    public void setControlsFlipped(boolean controlsFlipped) {
        isControlsFlipped = controlsFlipped;
    }

    public boolean isControlsFlipped() {
        return isControlsFlipped;
    }

    public void initDefaultCommands(Joystick joystick) {
        setDefaultCommand(new DriverControl(
                this,
                () -> joystick.getRawAxis(1),
                () -> joystick.getRawAxis(4)
        ));
    }

    @Override
    public void periodic() {
        double leftDist = leftEncoder.getDistance();
        double rightDist = rightEncoder.getDistance();

        // Update the odometry in the periodic block
        externalOdometry.update(Rotation2d.fromDegrees(getHeading()),
                leftDist,
                rightDist);

        internalOdometry.update(Rotation2d.fromDegrees(getHeading()),
                (leftNeoEncoder.getPosition() / 8.73) * 2 * Math.PI * DriveConstants.kWheelRadius,
                (leftNeoEncoder.getPosition() / 8.73) * 2 * Math.PI * DriveConstants.kWheelRadius);

        live_dashboard.getEntry("robotX").setDouble(Units.metersToFeet(getPose().getTranslation().getX()));
        live_dashboard.getEntry("robotY").setDouble(Units.metersToFeet(getPose().getTranslation().getY()));
        live_dashboard.getEntry("robotHeading").setDouble(getPose().getRotation().getRadians());

        SmartDashboard.putNumber("robotX", Units.metersToFeet(getPose().getTranslation().getX()));
        SmartDashboard.putNumber("robotY", Units.metersToFeet(getPose().getTranslation().getY()));
        SmartDashboard.putNumber("robotHeading", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("Internal RobotX", Units.metersToFeet(internalOdometry.getPoseMeters().getTranslation().getX()));
        SmartDashboard.putNumber("Internal RobotY", Units.metersToFeet(internalOdometry.getPoseMeters().getTranslation().getY()));

        SmartDashboard.putNumber("Left Encoder = ", leftEncoder.getDistance());
        SmartDashboard.putNumber("Right Encoder = ", rightEncoder.getDistance());
    }

}
