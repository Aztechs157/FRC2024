// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSystem;

public class DriveToSpeaker extends Command {
    private DriveSubsystem driveSystem;
    private VisionSystem visionSystem;

    /** Creates a new DriveTospeakerV2. */
    public DriveToSpeaker(DriveSubsystem driveSystem, VisionSystem visionSystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSystem);

        this.driveSystem = driveSystem;
        this.visionSystem = visionSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /**
         * TODO: This is likely not going to work perfectly first try. Things that could
         * be wrong:
         * 1) drive may want smaller values, currently just a P loop with a P value of
         * one, maybe try using properly tuned PID loop, trying to get all values to 0?
         * 2) the position of SHOOTING_POS_SPEAKER may be wrong, it may be trying to get
         * behind the speaker instead of in front of it. try making the value negative
         * if it overshoots
         * 3) double check camera transform3d. If it's wrong, then it will make
         * everything wrong, and we don't want that
         * 4) This may be trying to turn the robot 180 degrees away from the target
         * position. Try adding a 180 degree rotation to SHOOTING POS SPEAKER
         */
        var speakerTag = visionSystem.findSpeakerTag();
        if (speakerTag != null) {
            var speakerPos = speakerTag.getBestCameraToTarget();
            speakerPos = VisionConstants.CAMERA_PLACEMENT.plus(speakerPos);
            var speakerPos2d = new Transform2d(speakerPos.getX(), speakerPos.getY(),
                    new Rotation2d(speakerPos.getRotation().getZ()));
            var targetPos = speakerPos2d.plus(VisionConstants.SHOOTING_POS_SPEAKER);

            driveSystem.drive(targetPos.getTranslation(), targetPos.getRotation().getDegrees(), true);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
