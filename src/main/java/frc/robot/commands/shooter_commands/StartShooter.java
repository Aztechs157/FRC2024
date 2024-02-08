// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter_commands;

import org.assabet.aztechs157.numbers.Range;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSystem;

public class StartShooter extends Command {

    private final ShooterSystem shooterSystem;

    /** Creates a new StartShooter. */
    public StartShooter(final ShooterSystem shooterSystem) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.shooterSystem = shooterSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final var currentLeftMotorSet = shooterSystem.leftMotorPID(ShooterConstants.SHOOTER_TARGET_RPM);
        final var currentRightMotorSet = shooterSystem.rightMotorPID(ShooterConstants.SHOOTER_TARGET_RPM);
        shooterSystem.setLeftMotor(currentLeftMotorSet);
        shooterSystem.setRightMotor(currentRightMotorSet);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Shooter Spun Up");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        final var range = new Range(
                ShooterConstants.SHOOTER_TARGET_RPM - ShooterConstants.SHOOTER_START_VELOCITY_TOLERANCE,
                ShooterConstants.SHOOTER_TARGET_RPM + ShooterConstants.SHOOTER_START_VELOCITY_TOLERANCE);

        final var leftInRange = range.contains(shooterSystem.getLeftEncoderVelocity());
        final var rightInRange = range.contains(shooterSystem.getRightEncoderVelocity());

        return leftInRange && rightInRange;
    }
}
