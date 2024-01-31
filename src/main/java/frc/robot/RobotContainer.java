// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive_commands.TeleopDrive;
import frc.robot.inputs.Inputs;
import frc.robot.subsystems.DriveSystem;

public class RobotContainer {
    private final DriveSystem driveSystem = new DriveSystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final Inputs inputs = Inputs.createFromChooser();

    public RobotContainer() {
        driveSystem.setDefaultCommand(new TeleopDrive(driveSystem, inputs));

        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
