// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import frc.robot.drive.Angle_Step_Data;
import frc.robot.drive.Drive;
import frc.robot.drive.TeleopDrive;
import frc.robot.inputs.Inputs;

public class RobotContainer {
    private final Drive drive = new Drive();
    private final Inputs inputs = Inputs.createFromChooser();

    public RobotContainer() {
        // drive.setDefaultCommand(new Angle_Step_Data());
        drive.setDefaultCommand(new TeleopDrive(drive, inputs));

        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
