// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hanger_commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PneumaticsSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class ExtendHangerPin extends InstantCommand {

    private final PneumaticsSystem pneumaticsSystem;

    public ExtendHangerPin(final PneumaticsSystem pneumaticsSystem) {
        this.pneumaticsSystem = pneumaticsSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pneumaticsSystem.deployHangerPin(DoubleSolenoid.Value.kForward);
    }
}
