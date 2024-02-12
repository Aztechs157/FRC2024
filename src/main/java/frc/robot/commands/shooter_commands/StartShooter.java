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
    private final double setPoint;

    /** Creates a new StartShooter. */
    public StartShooter(final ShooterSystem shooterSystem, final double setPoint) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.setPoint = setPoint;
        this.shooterSystem = shooterSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterSystem.currentLeftMotorSet += shooterSystem.leftMotorPID(setPoint);
        shooterSystem.currentRightMotorSet += shooterSystem.rightMotorPID(setPoint);
        shooterSystem.setLeftMotor(shooterSystem.currentLeftMotorSet);
        shooterSystem.setRightMotor(shooterSystem.currentRightMotorSet);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Shooter Spun Up");
        if (interrupted) {
            shooterSystem.setLeftMotor(0);
            shooterSystem.setRightMotor(0);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        final var range = new Range(
                setPoint - ShooterConstants.SHOOTER_START_VELOCITY_TOLERANCE,
                setPoint + ShooterConstants.SHOOTER_START_VELOCITY_TOLERANCE);

        final var leftInRange = range.contains(-shooterSystem.getLeftEncoderVelocity());
        final var rightInRange = range.contains(shooterSystem.getRightEncoderVelocity());

        return leftInRange && rightInRange;
        // return false;

    }
}
