package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriverControl;

public class Drivetrain extends SubsystemBase {
    // motors on left side of drive
    private SpeedControllerGroup leftMotors;
    private SpeedControllerGroup rightMotors;

    // motors
    private CANSparkMax lMotor0;
    private CANSparkMax lMotor1;
    private CANSparkMax lMotor2;
    private CANSparkMax rMotor0;
    private CANSparkMax rMotor1;
    private CANSparkMax rMotor2;

    // Encoders
    private Encoder leftEncoder;
    private Encoder rightEncoder;

    // Gyro
    private AHRS navx;


    private DifferentialDrive drive;

    private DifferentialDriveOdometry odometry;

    public Drivetrain() {
        lMotor0 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        lMotor1 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        lMotor2 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        rMotor0 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
        rMotor1 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
        rMotor2 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);

        lMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
        lMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        lMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        rMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        lMotor0.setSmartCurrentLimit(50);
        lMotor1.setSmartCurrentLimit(50);
        lMotor2.setSmartCurrentLimit(50);

        rMotor0.setSmartCurrentLimit(50);
        rMotor1.setSmartCurrentLimit(50);
        rMotor2.setSmartCurrentLimit(50);

        leftMotors = new SpeedControllerGroup(lMotor0,lMotor1,lMotor2);
        rightMotors = new SpeedControllerGroup(rMotor0, rMotor1,rMotor2);

        leftEncoder = new Encoder(
                DriveConstants.kLeftEncoderPorts[0],
                DriveConstants.kRightEncoderPorts[1],
                DriveConstants.kLeftEncoderReversed
        );

        rightEncoder = new Encoder(
                DriveConstants.kRightEncoderPorts[0],
                DriveConstants.kRightEncoderPorts[1],
                DriveConstants.kRightEncoderReversed
        );

        navx = new AHRS(SPI.Port.kMXP);

        drive = new DifferentialDrive(lMotor0, rMotor0);
        drive.setSafetyEnabled(false);

        resetEncoders();
        // TODO: set encoder distance per pulse
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
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
        return Math.IEEEremainder(navx.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setTankDrive(double leftVel, double rightVel) {
        drive.tankDrive(leftVel, rightVel);
    }

    // for ramsete
    public void setDriveVolts(double lVolts, double rVolts) {
        leftMotors.setVoltage(lVolts);
        rightMotors.setVoltage(-rVolts);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    @Override
    public void periodic() {
        // TODO: populate this with A LOT more (logging ect)
        odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(),
                rightEncoder.getDistance());
    }

}
