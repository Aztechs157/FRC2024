// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.EmptyCommand;
import frc.robot.commands.drive_commands.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.hanger_commands.ExtendHanger;
import frc.robot.commands.hanger_commands.RetractHanger;
import frc.robot.commands.intake_commands.Eject;
import frc.robot.commands.intake_commands.Intake;
import frc.robot.commands.intake_commands.LoadNote;
import frc.robot.commands.shooter_commands.DeployDeflector;
import frc.robot.commands.shooter_commands.RetractDeflector;
import frc.robot.commands.shooter_commands.Shoot;
import frc.robot.commands.shooter_commands.SpinUpShooter;
import frc.robot.commands.shooter_commands.StartShooter;
import frc.robot.commands.shooter_commands.StartUpShooterCommanded;
import frc.robot.commands.shooter_commands.commandShoot;
// import frc.robot.commands.vision_commands.VisionPoseEstimator;
import frc.robot.cosmetics.PwmLEDs;
import frc.robot.inputs.Inputs;
import frc.robot.subsystems.DeflectorSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.HangerSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.LogicSystem;
import frc.robot.subsystems.ShooterSystem;
// import frc.robot.subsystems.VisionSystem;
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
    public final SystemConfigJson systemConfigs = new ConfigParser(isBeta.get()).systemConfigJson;

    // The robot's subsystems and commands are defined here...
    private final DriveSystem drivebase = new DriveSystem(new File(Filesystem.getDeployDirectory(),
            isBeta.get() ? "beta/swerve" : "alpha/swerve"), isBeta.get(), Constants.DEBUG_MODE);
    private final Inputs inputs = Inputs.createFromChooser();
    // private final PneumaticsSystem pneumaticsSystem = new
    // PneumaticsSystem(isBeta.get());
    private final IntakeSystem intakeSystem;
    private final ShooterSystem shooterSystem;

    private final LogicSystem logicSystem;

    private final DeflectorSystem deflectorSystem;
    public final HangerSystem hangerSystem;
    // public final VisionSystem visionSystem;
    public final PwmLEDs lightSystem;
    private final SendableChooser<Command> autoChooser;
    public static int fieldOrientation = 1;

    XboxController driverXbox = new XboxController(0);

    public Command resetGyroCommand() {
        return drivebase.zeroHeading();
    }

    public Command reverseGyroCommand() {
        return drivebase.setGyroOffset(180);
    }

    public Command intakeCommand() {
        return new Intake(intakeSystem, lightSystem).andThen(new LoadNote(intakeSystem, lightSystem));
    }

    public Command loadNoteCommand() {
        return new LoadNote(intakeSystem, lightSystem);
    }

    public Command highShootSpinUpCommand() {
        if (!shooterSystem.getShootIsRunning()) {
            return new SpinUpShooter(shooterSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_HIGH);
        } else {
            return new EmptyCommand();
        }
    }

    public Command lowShootSpinUpCommand() {
        if (!shooterSystem.getShootIsRunning()) {
            return new SpinUpShooter(shooterSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_LOW);
        } else {
            return new EmptyCommand();
        }
    }

    // THIS ONE WORKS, DON'T CHANGE
    // public Command highShootCommand() {
    // return new StartShooter(shooterSystem, lightSystem,
    // ShooterConstants.SHOOTER_TARGET_RPM_HIGH)
    // .andThen(new Shoot(shooterSystem, intakeSystem, lightSystem,
    // ShooterConstants.SHOOTER_TARGET_RPM_HIGH));
    // }

    public Command lowShootCommand() {
        return new DeployDeflector(deflectorSystem)
                .andThen(new StartShooter(shooterSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_LOW))
                .andThen(new Shoot(shooterSystem, intakeSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_LOW))
                .finallyDo(new RetractDeflector(deflectorSystem)::schedule);
    }

    public Command highShootCommand() {
        return new StartUpShooterCommanded(shooterSystem, intakeSystem, logicSystem, lightSystem,
                ShooterConstants.SHOOTER_TARGET_RPM_HIGH);
    }

    public Command commandShootCommand() {
        return new commandShoot(logicSystem);
    }

    public Command passCommand() {
        return new StartShooter(shooterSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_PASS)
                .andThen(new Shoot(shooterSystem, intakeSystem, lightSystem, ShooterConstants.SHOOTER_TARGET_RPM_PASS));
    }

    public Command ejectCommand() {
        return new Eject(intakeSystem, shooterSystem, lightSystem);
    }

    public Command liftHangerCommand() {
        return new ExtendHanger(hangerSystem, lightSystem);
    }

    public Command retractHangerCommand() {
        return new RetractHanger(hangerSystem, lightSystem);
    }

    public double modifySpeed(final double speed) {
        final var modifier = 1 - inputs.axis(Inputs.precisionDrive).get();
        return speed * modifier;
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        if (systemConfigs.activeHanger) {
            hangerSystem = new HangerSystem(Constants.DEBUG_MODE);
        } else {
            hangerSystem = null;
        }

        // if (systemConfigs.activeVision) {
        // visionSystem = new VisionSystem();
        // } else {
        // visionSystem = null;
        // }

        if (systemConfigs.activeDriveCam) {
            Shuffleboard.getTab("Driver").add(CameraServer.startAutomaticCapture());
        }

        if (systemConfigs.activeLights) {
            lightSystem = new PwmLEDs();
        } else {
            lightSystem = null;
        }

        if (systemConfigs.activeIntake) {
            intakeSystem = new IntakeSystem();
        } else {
            intakeSystem = null;
        }

        if (systemConfigs.activeShooter) {
            shooterSystem = new ShooterSystem(isBeta.get(), Constants.DEBUG_MODE);

        } else {
            shooterSystem = null;
        }

        if (systemConfigs.activeDeflector) {
            deflectorSystem = new DeflectorSystem(Constants.DEBUG_MODE);

        } else {
            deflectorSystem = null;
        }

        logicSystem = new LogicSystem();

        // Register Named Commands
        NamedCommands.registerCommand("ResetGyro", resetGyroCommand());

        NamedCommands.registerCommand("Intake", intakeCommand());
        NamedCommands.registerCommand("LoadNote", loadNoteCommand());

        NamedCommands.registerCommand("HighShootSpinUp", highShootSpinUpCommand());
        NamedCommands.registerCommand("LowShootSpinUp", lowShootSpinUpCommand());

        NamedCommands.registerCommand("HighShoot", highShootCommand().withTimeout(2));
        NamedCommands.registerCommand("LowShoot", lowShootCommand().withTimeout(2));

        autoChooser = AutoBuilder.buildAutoChooser("NothingAuto");

        autoChooser.onChange((command) -> {
            System.out.println("Autonomous routine changed to: " + command.getName());
        });

        Shuffleboard.getTab("Auton").add(autoChooser);

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
                () -> modifySpeed(
                        MathUtil.applyDeadband(fieldOrientation * driverXbox.getLeftY(),
                                ControllerConstants.LEFT_Y_DEADBAND)),
                () -> modifySpeed(
                        MathUtil.applyDeadband(fieldOrientation * driverXbox.getLeftX(),
                                ControllerConstants.LEFT_X_DEADBAND)),
                () -> modifySpeed(-driverXbox.getRightX()));

        Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
                () -> -driverXbox.getRightX());

        drivebase.setDefaultCommand(
                !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

        // visionSystem.setDefaultCommand(new VisionPoseEstimator(visionSystem,
        // drivebase));
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

        if (systemConfigs.activeIntake) {
            inputs.button(Inputs.intake).toggleWhenPressed(intakeCommand());
            inputs.button(Inputs.loadNote).whenPressed(new LoadNote(intakeSystem, lightSystem));
        }

        if (systemConfigs.activeIntake && systemConfigs.activeShooter) {
            inputs.button(Inputs.highShotSpinUp)
                    .toggleWhenPressed(highShootCommand().until(logicSystem::getCommandShoot));
            inputs.button(Inputs.highShot).toggleWhenPressed(commandShootCommand());

            inputs.button(Inputs.pass).toggleWhenPressed(passCommand());
            inputs.button(Inputs.eject).toggleWhenPressed(ejectCommand());

            if (systemConfigs.activeDeflector) {
                inputs.button(Inputs.lowShotSpinUp).toggleWhenPressed(lowShootSpinUpCommand());
                inputs.button(Inputs.lowShot).toggleWhenPressed(lowShootCommand());
            }
        }

        if (systemConfigs.activeHanger) {
            inputs.button(Inputs.liftHanger).toggleWhenPressed(liftHangerCommand());
            inputs.button(Inputs.retractHanger).toggleWhenPressed(retractHangerCommand());
        }

        inputs.button(Inputs.resetGyro).whenPressed(resetGyroCommand());

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
