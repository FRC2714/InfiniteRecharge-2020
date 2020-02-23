/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.auto.*;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.shooter.TeleopShooter;
import frc.robot.subsystems.*;
import frc.robot.commands.drivetrain.AlignToTarget;
import frc.robot.commands.drivetrain.DriverControl;
import frc.robot.utils.Logger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private final Limelight limelight = new Limelight();
    private final Shooter shooter = new Shooter(limelight);
    private final Conveyor conveyor = new Conveyor(shooter::atSetpoint);
    private final Intake intake = new Intake();
    private final Climber climber = new Climber();

    private final Drivetrain drivetrain = new Drivetrain();

    private static Joystick driverStick = new Joystick(0);
    private static Joystick operatorStick = new Joystick(1);

    private JoystickButton driverAButton = new JoystickButton(driverStick, 1);
    private JoystickButton driverBButton = new JoystickButton(driverStick, 2);
    private POVButton driverLeftPOVButton = new POVButton(driverStick, 90);

    private JoystickButton operatorAButton = new JoystickButton(operatorStick, 1);
    private JoystickButton operatorBButton = new JoystickButton(operatorStick, 2);
    private JoystickButton operatorLeftShoulder = new JoystickButton(operatorStick, 5);
    private JoystickButton operatorYButton = new JoystickButton(operatorStick, 4);
    private JoystickButton operatorXButton = new JoystickButton(operatorStick, 3);



    private DriverControl driverControlCommand = new DriverControl(
            drivetrain,
            () -> driverStick.getRawAxis(1),
            () -> driverStick.getRawAxis(4)
    );

    private Logger<Subsystem> subsystemLogger = new Logger<>();

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        initDefaultCommands();
    }

    public void initDefaultCommands() {
        drivetrain.initDefaultCommands(driverStick);
        conveyor.initDefaultCommand(shooter::atSetpoint);
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driverAButton.whileHeld(new AlignToTarget(limelight, drivetrain, () -> driverStick.getRawAxis(1)));
        driverBButton.whenPressed(new InstantCommand(() -> drivetrain.setControlsFlipped(!drivetrain.isControlsFlipped())));

        operatorAButton.whileHeld(new AutoIntake(shooter,intake, conveyor, AutoIntake.IntakeType.INTAKE));
        operatorBButton.whileHeld(new AutoIntake(shooter,intake, conveyor, AutoIntake.IntakeType.EXTAKE));
        operatorXButton.whileHeld(new AutoIntake(shooter, intake, conveyor, AutoIntake.IntakeType.SHOOT));

        operatorLeftShoulder.whileHeld(new TeleopShooter(shooter,conveyor,1000));
        operatorYButton.whileHeld(new InstantCommand(
                () -> climber.setPower(1.0)
        ));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getGeneratorAuto() {
        return new GeneratorAuto(drivetrain, limelight);
    }

    public Command getBallStealAutonomous() {
        return new BallStealAuto(drivetrain, intake, conveyor, shooter, limelight);
    }

    public Command getSideTrenchRunAuto(){
        return new SideTrenchRun(drivetrain, intake, conveyor, shooter, limelight);
    }

    public Command getTrenchRunAuto() {
        return new TrenchRunAuto(drivetrain, intake, conveyor, shooter, limelight);
    }

    public Command getSplineTestAuto(){return new SplineTesting(drivetrain,limelight);}

    public void clearMovingMotors(){
        shooter.disable();
        conveyor.disable();
        drivetrain.tankDriveVolts(0,0);
    }

}
