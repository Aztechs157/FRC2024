// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class ManualShoot extends Command {

    private final ShooterSystem shooterSystem;
    private final IntakeSystem intakeSystem;
    private boolean seenNote = false;

    /** Creates a new Shoot. */
    public ManualShoot(final ShooterSystem shooterSystem, final IntakeSystem intakeSystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeSystem);

        this.shooterSystem = shooterSystem;
        this.intakeSystem = intakeSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Run ManualShoot Command");
        intakeSystem.set(-IntakeConstants.INTAKE_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (intakeSystem.checkForNote()) {
            seenNote = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSystem.setMotors(0);
        intakeSystem.set(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return seenNote && !(intakeSystem.checkForNote());
    }
}
