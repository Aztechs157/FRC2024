// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSystem;

public class ManualShoot extends Command {
    private final ShooterSystem shooterSystem;

    /** Creates a new Shoot. */
    public ManualShoot(final ShooterSystem shooterSystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSystem);

        this.shooterSystem = shooterSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooterSystem.set(ShooterConstants.SHOOT_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSystem.set(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
