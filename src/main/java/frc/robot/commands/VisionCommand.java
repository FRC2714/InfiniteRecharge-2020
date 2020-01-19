package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.CameraConstants;

import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;

public class VisionCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private NetworkTable limelight;
    // private final BooleanSupplier buttonPressed;
    public static final double kCameraToGoalHeight = FieldConstants.kGoalHeight - CameraConstants.kCameraHeight;

    public VisionCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // this.buttonPressed = buttonPressed;
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getDistanceToGoal() {
        if (getYAngleOffset() == -1) return -1;
        return kCameraToGoalHeight / Math.tan(Math.toRadians(CameraConstants.kCameraMountingAngle + getYAngleOffset()));
    }

    public double getYAngleOffset() {
        return limelight.getEntry("ty").getDouble(-1);
    }

    public double getXAngleOffset() {
        return limelight.getEntry("tx").getDouble(-1);
    }

    public boolean targetVisible() {
        double tv = limelight.getEntry("tv").getDouble(0.0);
        if (tv != 0.0) return true;
        return false;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("cam to goal", kCameraToGoalHeight);
        SmartDashboard.putNumber("X offset", getXAngleOffset());
        SmartDashboard.putNumber("Y offset", getYAngleOffset());
        SmartDashboard.putNumber("Distance", getDistanceToGoal());
        double tx = getXAngleOffset();
        /*
        if (buttonPressed.getAsBoolean()) {
            double headingError = -tx;
            double adjust = 0.0;

            if (tx > 1.0)
                adjust = 0.5 * headingError -.05;
            else if (tx < 1.0)
                adjust = 0.5 * headingError + .05;

            drivetrain.arcadeDrive(0, adjust);
        }*/
    }

}
