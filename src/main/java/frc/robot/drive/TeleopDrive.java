// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.List;

import org.assabet.aztechs157.PrintLimiter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.inputs.Inputs;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class TeleopDrive extends Command {
    private final Drive drive;
    private final Inputs inputs;

    /** Creates a new TeleopDrive. */
    public TeleopDrive(final Drive drive, final Inputs inputs) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drive = drive;
        addRequirements(drive);
        this.inputs = inputs;
    }

    private final PrintLimiter limiter = new PrintLimiter(50);

    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // X and Y are swapped here due to trigonometry
        // Having them match will make the robot think forward is to it's right
        // Having them swapped will make it think forward is properly forward
        final var driveX = inputs.axis(Inputs.driveSpeedY).get();
        final var driveY = inputs.axis(Inputs.driveSpeedX).get();
        final var r = inputs.axis(Inputs.rotateSpeed).get();
        final var heading = new Rotation2d(r * Math.PI);

        final ChassisSpeeds speeds = drive.getTargetSpeeds(driveX, driveY, heading);

        // Limit velocity to prevent tippy
        Translation2d translation = SwerveController.getTranslation2d(speeds);
        translation = SwerveMath.limitVelocity(translation, drive.getFieldVelocity(), drive.getPose(),
                DriveConstants.LOOP_TIME, DriveConstants.ROBOT_MASS, List.of(DriveConstants.CHASSIS),
                drive.getSwerveDriveConfiguration());
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        limiter.tick().println("-".repeat(20)).println("x = " + driveX).println("y = " + driveY).println("r = " + r);

        drive.drive(translation, speeds.omegaRadiansPerSecond, true);
    }
}
