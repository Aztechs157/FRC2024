// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.VisionSystem;

public class VisionPoseEstimator extends Command {
    private VisionSystem visionSystem;
    private DriveSystem driveSystem;

    /** Creates a new VisionPoseEstimator. */
    public VisionPoseEstimator(final VisionSystem visionSystem, final DriveSystem driveSystem) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.visionSystem = visionSystem;
        this.driveSystem = driveSystem;

        addRequirements(visionSystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var pose = visionSystem.getEstimatedGlobalPose();
        if (pose.isPresent()) {
            driveSystem.addVisionReading(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
