// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drive_commands.TeleopDrive;
import frc.robot.inputs.Inputs;
import frc.robot.subsystems.DriveSystem;

public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final DriveSystem driveSystem = new DriveSystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final Inputs inputs = Inputs.createFromChooser();

    // CommandJoystick rotationController = new CommandJoystick(1);
    // CommandJoystick driverController = new CommandJoystick(1);
    // CommandJoystick driverController = new
    // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);

    XboxController driverXbox = new XboxController(Constants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        configureBindings();

        // driveSystem.setDefaultCommand(new TeleopDrive(driveSystem, inputs));

        /*
         * AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
         * () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
         * OperatorConstants.LEFT_Y_DEADBAND),
         * () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
         * OperatorConstants.LEFT_X_DEADBAND),
         * () -> MathUtil.applyDeadband(driverXbox.getRightX(),
         * OperatorConstants.RIGHT_X_DEADBAND),
         * driverXbox::getYButtonPressed,
         * driverXbox::getAButtonPressed,
         * driverXbox::getXButtonPressed,
         * driverXbox::getBButtonPressed);
         */

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation

        /*
         * Command driveFieldOrientedDirectAngle = driveSystem.driveCommand(
         * () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
         * DriveConstants.LEFT_Y_DEADBAND),
         * () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
         * DriveConstants.LEFT_X_DEADBAND),
         * () -> driverXbox.getRightX(),
         * () -> driverXbox.getRightY());
         */

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot

        Command driveFieldOrientedAnglularVelocity = driveSystem.driveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), DriveConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), DriveConstants.LEFT_X_DEADBAND),
                () -> driverXbox.getRightX());

        Command driveFieldOrientedDirectAngleSim = driveSystem.simDriveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                        DriveConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                        DriveConstants.LEFT_X_DEADBAND),
                () -> driverXbox.getRightX());

        driveSystem.setDefaultCommand(
                !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the
     * named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        /*
         * new JoystickButton(driverXbox, 1).onTrue((new
         * InstantCommand(driveSystem::zeroGyro)));
         * new JoystickButton(driverXbox, 3).onTrue(new
         * InstantCommand(driveSystem::addFakeVisionReading));
         * new JoystickButton(driverXbox,
         * 2).whileTrue(
         * Commands.deferredProxy(() -> driveSystem.driveToPose(
         * new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
         * // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new
         * // InstantCommand(drivebase::lock, drivebase)));
         */
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");

        // An example command will be run in autonomous
        // return drivebase.getAutonomousCommand("New Path", true);
    }

    public void setDriveMode() {
        // driveSystem.setDefaultCommand();
    }

    public void setMotorBrake(boolean brake) {
        driveSystem.setMotorBrake(brake);
    }
}
