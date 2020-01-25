package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

import java.util.function.BooleanSupplier;

public class AlignToTarget extends ProfiledPIDCommand {
    private Limelight limelight;

    public AlignToTarget(Limelight limelight, Drivetrain drive) {
        super(
                new ProfiledPIDController(DriveConstants.kAlignP,0, DriveConstants.kAlignD,
                        new TrapezoidProfile.Constraints(100, 300)),
                // Close loop on heading
                limelight::getXAngleOffset,
                // set reference to target
                0,
                // pipe to turn robot
                (output, setpoint) -> drive.arcadeDrive(0, output),
                // require drive
                drive
        );

        this.limelight = limelight;
        getController().enableContinuousInput(-180, 180);

        // TODO: set tolerances
        getController().setTolerance(.75, 3);
    }


    @Override
    public boolean isFinished() {
        System.out.println(getController().getPositionError());
        return
                getController().atGoal();
    }

}
