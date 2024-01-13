// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class Drive extends SubsystemBase {
    SwerveDrive swerve;

    /** Creates a new Drive. */
    public Drive() {
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(3.5), 8.14, 42);
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(1, 360);

        try {
            swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(17.3, angleConversionFactor, driveConversionFactor);
        } catch (IOException exception) {
            throw new RuntimeException("missing swerve json files"); // TODO: handle exception better
        }

        // setupPathPlanner();
    }

    public void set(ChassisSpeeds velocity) {
        swerve.drive(velocity);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
