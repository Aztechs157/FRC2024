// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands.auto;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.cosmetics.PwmLEDs;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.VisionSystem;

public class DriveToAmp extends Command {

    private final VisionSystem visionSystem;
    private final DriveSystem driveBase;
    private final PwmLEDs lightSystem;

    private PhotonTrackedTarget target;
    private Transform3d pathToTarget;

    /** Creates a new DriveToAmp. */
    public DriveToAmp(final VisionSystem visionSystem, final DriveSystem driveBase, final PwmLEDs lightSystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveBase);

        this.visionSystem = visionSystem;
        this.driveBase = driveBase;
        this.lightSystem = lightSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        target = visionSystem.findAmpTag();
        pathToTarget = visionSystem.getBestPathToTarget(target);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
