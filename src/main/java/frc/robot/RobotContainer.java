// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.assabet.aztechs157.input.models.XboxOne;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.legacy_drive.LegacyDrive;

public class RobotContainer {
    public XboxOne inputs = new XboxOne(0);

    public LegacyDrive legacyDrive = new LegacyDrive();

    public RobotContainer() {
        configureDefaults();
        configureBindings();
    }

    private void configureDefaults() {
        legacyDrive.setDefaultCommand(legacyDrive.fullDrive(inputs));
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
