// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drive_commands.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.hanger_commands.ExtendHangerPin;
import frc.robot.commands.hanger_commands.BuildHangerTension;
import frc.robot.commands.hanger_commands.LiftHanger;
import frc.robot.commands.hanger_commands.RetractHanger;
import frc.robot.commands.hanger_commands.RetractHangerPin;
import frc.robot.commands.intake_commands.Intake;
import frc.robot.commands.intake_commands.LoadNote;
import frc.robot.commands.shooter_commands.Shoot;
import frc.robot.commands.shooter_commands.SpinUpShooter;
import frc.robot.commands.shooter_commands.StartShooter;
import frc.robot.cosmetics.PwmLEDs;
import frc.robot.inputs.Inputs;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.HangerSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.PneumaticsSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.VisionSystem;
import java.io.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final DigitalInput isBeta = new DigitalInput(9);

    // The robot's subsystems and commands are defined here...
    private final DriveSystem drivebase = new DriveSystem(new File(Filesystem.getDeployDirectory(),
            isBeta.get() ? "beta/swerve" : "alpha/swerve"), isBeta.get());
    private final Inputs inputs = Inputs.createFromChooser();
    private final PneumaticsSystem pneumaticsSystem = new PneumaticsSystem(isBeta.get());
    private final IntakeSystem intakeSystem = new IntakeSystem();
    private final ShooterSystem shooterSystem = new ShooterSystem(isBeta.get());
    public final HangerSystem hangerSystem;
    public final VisionSystem visionSystem = new VisionSystem();
    public final PwmLEDs lightSystem = new PwmLEDs();
    private final SendableChooser<Command> autoChooser;

    XboxController driverXbox = new XboxController(0);

    public Command intakeCommand() {
        return new Intake(intakeSystem, lightSystem)
                .alongWith(pneumaticsSystem.setIntakeFoward())
                .andThen(new LoadNote(intakeSystem, lightSystem).alongWith(pneumaticsSystem.setIntakeReverse()))
                .finallyDo(pneumaticsSystem.setIntakeReverse()::initialize);
    }

    public Command highShootSpinUpCommand() {
        return new SpinUpShooter(shooterSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_HIGH);
    }

    public Command lowShootSpinUpCommand() {
        return new SpinUpShooter(shooterSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_LOW);
    }

    public Command highShootCommand() {
        return new StartShooter(shooterSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_HIGH)
                .andThen(new Shoot(shooterSystem, intakeSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_HIGH));
    }

    public Command lowShootCommand() {
        return pneumaticsSystem.setDeflectorFoward()
                .andThen(new StartShooter(shooterSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_LOW))
                .andThen(new Shoot(shooterSystem, intakeSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_LOW))
                .andThen(pneumaticsSystem.setDeflectorReverse())
                .finallyDo(pneumaticsSystem.setDeflectorReverse()::initialize);
    }

    public Command passCommand() {
        return new StartShooter(shooterSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_PASS)
                .andThen(new Shoot(shooterSystem, intakeSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_PASS));
    }

    public Command liftHangerCommand() {
        return new BuildHangerTension(hangerSystem)
                .andThen(new RetractHangerPin(pneumaticsSystem, hangerSystem)
                        .andThen(new LiftHanger(hangerSystem, lightSystem)));
    }

    public Command retractHangerCommand() {
        return new RetractHanger(hangerSystem, lightSystem).andThen(new ExtendHangerPin(pneumaticsSystem));
    }

    public double modifySpeed(final double speed) {
        final var slowed = inputs.button(Inputs.slowDriveSpeed).get();
        final var modifier = slowed ? 0.5 : 1;
        return speed * modifier;
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        if (isBeta.get()) {
            hangerSystem = new HangerSystem();
            Shuffleboard.getTab("Driver").add(CameraServer.startAutomaticCapture());
        } else {
            hangerSystem = null;
        }

        // Register Named Commands
        NamedCommands.registerCommand("Intake", intakeCommand());

        NamedCommands.registerCommand("HighShootSpinUp", highShootSpinUpCommand());
        NamedCommands.registerCommand("LowShootSpinUp", lowShootSpinUpCommand());

        NamedCommands.registerCommand("HighShoot", highShootCommand().withTimeout(4));
        NamedCommands.registerCommand("LowShoot", lowShootCommand().withTimeout(4));

        autoChooser = AutoBuilder.buildAutoChooser("NothingAuto");

        autoChooser.onChange((command) -> {
            System.out.println("Autonomous routine changed to: " + command.getName());
        });

        Shuffleboard.getTab("Driver").add(autoChooser);

        // Configure the trigger bindings
        configureBindings();

        AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                        ControllerConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                        ControllerConstants.LEFT_X_DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                        ControllerConstants.RIGHT_X_DEADBAND),
                driverXbox::getYButtonPressed,
                driverXbox::getAButtonPressed,
                driverXbox::getXButtonPressed,
                driverXbox::getBButtonPressed);

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
                () -> driverXbox.getRightX(),
                () -> driverXbox.getRightY());

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
                () -> modifySpeed(MathUtil.applyDeadband(driverXbox.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND)),
                () -> modifySpeed(MathUtil.applyDeadband(driverXbox.getLeftX(), ControllerConstants.LEFT_X_DEADBAND)),
                () -> driverXbox.getRightX());

        Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
                () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
                () -> -driverXbox.getRightX());

        drivebase.setDefaultCommand(
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
         * InstantCommand(drivebase::zeroGyro)));
         * new JoystickButton(driverXbox, 3).onTrue(new
         * InstantCommand(drivebase::addFakeVisionReading));
         * new JoystickButton(driverXbox,
         * 2).whileTrue(
         * Commands.deferredProxy(() -> drivebase.driveToPose(
         * new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
         * // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new
         * // InstantCommand(drivebase::lock, drivebase)));
         */

        inputs.button(Inputs.intake).toggleWhenPressed(intakeCommand());
        inputs.button(Inputs.loadNote).whenPressed(new LoadNote(intakeSystem, lightSystem));

        inputs.button(Inputs.highShotSpinUp).toggleWhenPressed(highShootSpinUpCommand());
        inputs.button(Inputs.lowShotSpinUp).toggleWhenPressed(lowShootSpinUpCommand());

        inputs.button(Inputs.highShot).toggleWhenPressed(highShootCommand());
        inputs.button(Inputs.lowShot).toggleWhenPressed(lowShootCommand());
        inputs.button(Inputs.pass).toggleWhenPressed(passCommand());

        if (isBeta.get()) {
            inputs.button(Inputs.liftHanger).toggleWhenPressed(liftHangerCommand());
            inputs.button(Inputs.retractHanger).toggleWhenPressed(retractHangerCommand());
            inputs.button(Inputs.retractHangerPin).whenPressed(new RetractHangerPin(pneumaticsSystem, hangerSystem));
            inputs.button(Inputs.extendHangerPin).whenPressed(new ExtendHangerPin(pneumaticsSystem));
        }

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }

    public void setDriveMode() {
        // drivebase.setDefaultCommand();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
