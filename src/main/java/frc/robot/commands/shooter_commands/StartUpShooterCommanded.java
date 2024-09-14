// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter_commands;

import org.assabet.aztechs157.numbers.Range;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.cosmetics.PwmLEDs;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.LogicSystem;

public class StartUpShooterCommanded extends Command {

    private final ShooterSystem shooterSystem;
    private final IntakeSystem intakeSystem;
    private final LogicSystem logicSystem;
    private final PwmLEDs lightSystem;
    private final double setPoint;

    private boolean seenNote = false;
    private Timer timer = new Timer();
    private boolean shooterActive = false;
    private final Range range;

    /** Creates a new StartShooter. */
    public StartUpShooterCommanded(final ShooterSystem shooterSystem, final IntakeSystem intakeSystem,
            final LogicSystem logicSystem, final PwmLEDs lightSystem, final double setPoint) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSystem, intakeSystem);

        this.shooterSystem = shooterSystem;
        this.intakeSystem = intakeSystem;
        this.logicSystem = logicSystem;
        this.lightSystem = lightSystem;
        this.setPoint = setPoint;

        range = new Range(
                setPoint - ShooterConstants.SHOOTER_START_VELOCITY_TOLERANCE,
                setPoint + ShooterConstants.SHOOTER_START_VELOCITY_TOLERANCE);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooterActive = true;

        lightSystem.setClimb(Color.kRed, Color.kBlack, 3, 7, 2);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // int colorLength = (int) Math.floor(10 *
        // shooterSystem.getLeftEncoderVelocity() / setPoint);

        System.out.println("IGJJJHJJRG");

        shooterSystem.currentLeftMotorSet += shooterSystem.leftMotorPID(setPoint);
        shooterSystem.currentRightMotorSet += shooterSystem.rightMotorPID(setPoint);
        shooterSystem.setLeftMotor(shooterSystem.currentLeftMotorSet);
        shooterSystem.setRightMotor(shooterSystem.currentRightMotorSet);

        final var leftInRange = range.contains(Math.abs(shooterSystem.getLeftEncoderVelocity()));
        final var rightInRange = range.contains(Math.abs(shooterSystem.getRightEncoderVelocity()));

        if (leftInRange && rightInRange) {
            lightSystem.setSolid(Color.kRed);
        }

        System.out.println("logic");

        // lightSystem.setClimb(Color.kRed, Color.kBlack, colorLength, 10 - colorLength,
        // 2);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println(interrupted);
        System.out.println(logicSystem.commandShoot);
        if (interrupted && !logicSystem.commandShoot) {
            shooterSystem.setLeftMotor(0);
            shooterSystem.setRightMotor(0);
            lightSystem.setDefault();
        } else if (logicSystem.commandShoot) {
            lightSystem.setSolid(Color.kYellow);
            intakeSystem.set(-IntakeConstants.FEED_SPEED);
            timer.reset();

            if (!(intakeSystem.checkForNote())) {
                timer.start();
            }

            // while (!timer.hasElapsed(ShooterConstants.SHOOTER_SENSOR_WAIT_TIME)) {
            // System.out.println("Help I'm Stuck");
            // shooterSystem.currentLeftMotorSet +=
            // shooterSystem.leftMotorPID(this.setPoint);
            // shooterSystem.currentRightMotorSet +=
            // shooterSystem.rightMotorPID(this.setPoint);
            // shooterSystem.setLeftMotor(shooterSystem.currentLeftMotorSet);
            // shooterSystem.setRightMotor(shooterSystem.currentRightMotorSet);
            // }

            if (timer.get() <= ShooterConstants.SHOOTER_SENSOR_WAIT_TIME) {
                shooterSystem.currentLeftMotorSet = 0;
                shooterSystem.currentRightMotorSet = 0;
                shooterSystem.setShooterMotors(0);
                intakeSystem.set(0);
                lightSystem.setDefault();
                shooterSystem.setShootIsRunning(false);
            }

            logicSystem.setCommandShoot(false);
            intakeSystem.hasNote = false;
            shooterActive = false;
        } else {
            shooterSystem.setLeftMotor(0);
            shooterSystem.setRightMotor(0);
            lightSystem.setStrobe(Color.kHotPink, Color.kAqua, 1, 0.5);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !logicSystem.commandShoot && !shooterActive;
    }
}
