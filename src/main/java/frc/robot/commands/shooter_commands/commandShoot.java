// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LogicSystem;

public class commandShoot extends Command {

    private final LogicSystem logicSystem;

    /** Creates a new commandShoot. */
    public commandShoot(final LogicSystem logicSystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(logicSystem);

        this.logicSystem = logicSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logicSystem.commandShoot = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        logicSystem.commandShoot = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logicSystem.commandShoot = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
