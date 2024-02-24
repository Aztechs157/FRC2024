// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;

import java.io.File;
import java.io.IOException;
import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    private static Robot instance;
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private Timer disabledTimer;

    private int poseEstimatorCounter = 0;

    private MechanismLigament2d shooterWheel1;
    private MechanismLigament2d shooterWheel2;

    public Robot() {
        instance = this;
    }

    public static Robot getInstance() {
        return instance;
    }

    private void estimatePose() {
        poseEstimatorCounter++;
        if (poseEstimatorCounter >= DriveConstants.POSE_ESTIMATE_FREQUENCY) {
            m_robotContainer.visionSystem.getEstimatedGlobalPose(null); // TODO: figure out how to get prevRobotPose
            poseEstimatorCounter = 0;
        }
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        // Create a timer to disable motor brake a few seconds after disable. This will
        // let the robot stop
        // immediately when disabled, but then also let it be pushed more
        disabledTimer = new Timer();

        // the main mechanism object
        Mechanism2d shooter = new Mechanism2d(3, 3);
        // the mechanism root node
        MechanismRoot2d root1 = shooter.getRoot("shooter_l", 2, 5);
        MechanismRoot2d root2 = shooter.getRoot("shooter_r", 8, 5);

        shooterWheel1 = root1.append(new MechanismLigament2d("shooterWheel1", 1, 0));
        shooterWheel2 = root2.append(new MechanismLigament2d("shooterWheel2", 1, 0));

        // post the mechanism to the dashboard
        SmartDashboard.putData("Mech2d", shooter);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want ran
     * during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        m_robotContainer.setMotorBrake(true);
        m_robotContainer.lightSystem.setDefault();
        m_robotContainer.visionSystem.updateAlliance();
    }

    @Override
    public void disabledPeriodic() {

    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_robotContainer.visionSystem.updateAlliance();
        m_robotContainer.setMotorBrake(true); // TODO: test the impact of brake mode on autonomous
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        // TODO: estimatePose();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        m_robotContainer.visionSystem.updateAlliance();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.setDriveMode();
        m_robotContainer.setMotorBrake(true);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        // TODO: estimatePose();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        try {
            new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
