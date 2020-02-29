/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer robotContainer;

    private SendableChooser<Command> autoChooser;


    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        autoChooser = new SendableChooser<>();


        autoChooser.setDefaultOption("Normal Trench Run Auto", robotContainer.getTrenchRunAuto());
        autoChooser.addOption("Side Trench Run Auto", robotContainer.getSideTrenchRunAuto());
        autoChooser.addOption("Ball Steal Auto", robotContainer.getBallStealAutonomous());
        autoChooser.addOption("Generator Auto", robotContainer.getGeneratorAuto());
        autoChooser.addOption("Shoot, Drive Forward", robotContainer.getCustomWaitAuto(0,false));
        autoChooser.addOption("Shoot, Drive Back", robotContainer.getCustomWaitAuto(0, true));
        autoChooser.addOption("5 delay, shoot, drive forward", robotContainer.getCustomWaitAuto(5, false));
        autoChooser.addOption("10 delay, shoot, drive back", robotContainer.getCustomWaitAuto(8, true));
        autoChooser.addOption("Do Nothing", robotContainer.getNothingAuto());

        SmartDashboard.putData("Auto Picker", autoChooser);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putString("Current Auto Picked", autoChooser.getSelected().getName());
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = autoChooser.getSelected();
        m_autonomousCommand.initialize();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }


    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        robotContainer.initTeleop();
        robotContainer.clearMovingMotors();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        robotContainer.initDefaultCommands();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
