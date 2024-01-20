// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import org.assabet.aztechs157.PrintLimiter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.inputs.Inputs;

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

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // X and Y are swapped here due to trigonometry
        // Having them match will make the robot think forward is to it's right
        // Having them swapped will make it think forward is properly forward
        final var x = inputs.axis(Inputs.driveSpeedY).get();
        final var y = inputs.axis(Inputs.driveSpeedX).get();
        final var r = inputs.axis(Inputs.rotateSpeed).get();

        limiter.tick().println("-".repeat(20)).println("x = " + x).println("y = " + y).println("r = " + r);

        final var speeds = new ChassisSpeeds(x, y, Math.toRadians(r));
        drive.set(speeds);
    }
}
