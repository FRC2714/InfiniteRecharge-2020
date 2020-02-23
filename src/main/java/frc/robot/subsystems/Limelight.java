package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    private double tx, ty, tv, distance = 0;

    private NetworkTable limelight;
    // private final BooleanSupplier buttonPressed;
    private static final double kCameraToGoalHeight =
            Constants.FieldConstants.kGoalHeight - Constants.CameraConstants.kCameraHeight;

    Spark ledShooter;

    public Limelight() {
        // this.buttonPressed = buttonPressed;
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        ledShooter = new Spark(0);
    }

    private double internalGetDistanceToGoal() {
        if (getYAngleOffset() == -1) return -1;
        return kCameraToGoalHeight / Math.tan(Math.toRadians(Constants.CameraConstants.kCameraMountingAngle + getYAngleOffset()));
    }

    public double getDistanceToGoal() {
        return distance;
    }

    public double getYAngleOffset() {
        return ty;
    }

    public double getXAngleOffset() {
        return -1 * tx;
    }

    public boolean targetVisible() {
        double tv = limelight.getEntry("tv").getDouble(0.0);
        return tv != 0.0;
    }

    public void setLED(boolean isBrightLightOn) {
        if (isBrightLightOn)
            limelight.getEntry("ledMode").setDouble(3);
        else
            limelight.getEntry("ledMode").setDouble(1);
    }

    public void setPipeline(double numPipeline) {
        limelight.getEntry("pipeline").setDouble(numPipeline);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Est. Distance (ft)", Units.metersToFeet(distance));

        tx = limelight.getEntry("tx").getDouble(-1);
        ty = limelight.getEntry("ty").getDouble(-1);
        tv = limelight.getEntry("tz").getDouble(0);
        distance = internalGetDistanceToGoal();

        if(targetVisible() && getXAngleOffset() < 1)
            ledShooter.set(0.71);
        else
            ledShooter.set(0.61);

    }
}
