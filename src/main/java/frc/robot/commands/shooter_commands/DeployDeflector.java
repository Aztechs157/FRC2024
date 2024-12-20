// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DeflectorSystem;

public class DeployDeflector extends Command {

    private final DeflectorSystem deflectorSystem;

    /** Creates a new DeployDeflector. */
    public DeployDeflector(final DeflectorSystem deflectorSystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(deflectorSystem);

        this.deflectorSystem = deflectorSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        deflectorSystem.set(deflectorSystem.deflectorMotorPID(95));
        System.out.println(deflectorSystem.deflectorMotorPID(95));
        System.out.println("working please");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        deflectorSystem.set(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (deflectorSystem.readDeflectorPotMapped() >= 95 && deflectorSystem.readDeflectorPotMapped() <= 98) {
            return true;
        } else {
            return false;
        }
        // return deflectorSystem.readExtLimitSwitch();

    }
}
