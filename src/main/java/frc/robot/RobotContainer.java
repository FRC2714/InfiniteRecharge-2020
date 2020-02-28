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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.*;
import frc.robot.commands.climber.MoveClimber;
import frc.robot.commands.conveyor.SingleShot;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.shooter.TeleopShooter;
import frc.robot.subsystems.*;
import frc.robot.commands.drivetrain.AlignToTarget;

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
    private JoystickButton driverLeftShoulder = new JoystickButton(driverStick, 5);

    private JoystickButton operatorAButton = new JoystickButton(operatorStick, 1);
    private JoystickButton operatorBButton = new JoystickButton(operatorStick, 2);
    private JoystickButton operatorLeftShoulder = new JoystickButton(operatorStick, 5);
    private JoystickButton operatorRightShoulder = new JoystickButton(operatorStick, 6);
    private JoystickButton operatorYButton = new JoystickButton(operatorStick, 4);
    private JoystickButton operatorXButton = new JoystickButton(operatorStick, 3);
    private JoystickButton operatorUnjamButton = new JoystickButton(operatorStick, 7);
    Trigger operatorForcedConveyorExtake = new Trigger(() -> operatorStick.getRawAxis(1) > 0.2);
    Trigger operatorForcedConveyorIntake = new Trigger(() -> operatorStick.getRawAxis(1) < -0.2);
    public Trigger operatorLeftTrigger = new Trigger(() -> operatorStick.getRawAxis(2) > 0.2);
    Trigger operatorRightTrigger = new Trigger(() -> operatorStick.getRawAxis(3) > 0.2);
    private POVButton operatorDPadUp = new POVButton(operatorStick, 0);
    private POVButton operatorDPadLeft = new POVButton(operatorStick, 90);
    private POVButton operatorDPadDown = new POVButton(operatorStick, 180);
    private POVButton operatorDPadRight = new POVButton(operatorStick, 270);



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
        driverLeftShoulder.whileHeld(new AutoIntake(shooter,intake, conveyor, AutoIntake.IntakeType.INTAKE));

//        operatorAButton.whileHeld(new AutoIntake(shooter,intake, conveyor, AutoIntake.IntakeType.INTAKE));
//        operatorBButton.whileHeld(new AutoIntake(shooter,intake, conveyor, AutoIntake.IntakeType.EXTAKE));
//        operatorXButton.whileHeld(new AutoIntake(shooter, intake, conveyor, AutoIntake.IntakeType.SHOOT));
//        operatorYButton.whileHeld(new SingleShot(shooter, intake, conveyor));
//
//        operatorLeftShoulder.whileHeld(new TeleopShooter(shooter,conveyor,1000));
//        operatorRightShoulder.whileHeld(new AutoIntake(shooter, intake, conveyor, AutoIntake.IntakeType.FORCED_CONVEYOR_INTAKE));

        operatorAButton.whileHeld(new AutoIntake(shooter,intake, conveyor, AutoIntake.IntakeType.INTAKE));
        operatorYButton.whileHeld(new AutoIntake(shooter,intake, conveyor, AutoIntake.IntakeType.EXTAKE));

        operatorForcedConveyorExtake
                .whileActiveContinuous(new AutoIntake(shooter, intake, conveyor, AutoIntake.IntakeType.FORCED_CONVEYOR_EXTAKE));

        operatorForcedConveyorIntake
                .whileActiveContinuous(new AutoIntake(shooter, intake, conveyor, AutoIntake.IntakeType.FORCED_CONVEYOR_INTAKE));

        operatorRightTrigger.whileActiveContinuous(new AutoIntake(shooter, intake, conveyor, AutoIntake.IntakeType.SHOOT));
        operatorUnjamButton.whileActiveContinuous(new AutoIntake(shooter, intake, conveyor, AutoIntake.IntakeType.UNJAM_STUCK_BALL));

        operatorLeftShoulder.or(operatorLeftTrigger)
                .whileActiveContinuous(new TeleopShooter(shooter,conveyor,1000));

        operatorRightShoulder.whileHeld(new SingleShot(shooter, intake, conveyor));

        operatorBButton.whileHeld(new MoveClimber(climber, MoveClimber.ClimberMotionType.MANUAL_EXTEND));
        operatorXButton.whileHeld(new MoveClimber(climber, MoveClimber.ClimberMotionType.MANUAL_RETRACT));

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
        return new NormalTrenchRunAuto(drivetrain, intake, conveyor, shooter, limelight);
    }

    public Command getSplineTestAuto(){return new SplineTesting(drivetrain,limelight);}

    public Command getCustomWaitAuto(double waitTime, boolean driveBackwards){
        return new WaitCommand(waitTime).andThen(new WaitShootAuton(drivetrain, intake, conveyor, shooter, limelight, driveBackwards));
    }

    public Command getNothingAuto(){
        return new InstantCommand(() -> drivetrain.tankDriveVolts(0,0));
    }


    public void clearMovingMotors(){
        shooter.disable();
        conveyor.disable();
        drivetrain.tankDriveVolts(0,0);
    }

}
