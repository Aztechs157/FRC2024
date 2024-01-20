// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.legacy_drive;

import org.assabet.aztechs157.SwervePod;
import org.assabet.aztechs157.input.models.XboxOne;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LegacyDriveConstants;

public class LegacyDrive extends SubsystemBase {
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            LegacyDriveConstants.WHEEL_LOCATIONS);

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("157/Swerve");

    public SwervePod[] swervePods = new SwervePod[] {
            new SwervePod(new OrignalSwervePodIO(LegacyDriveConstants.POD_CONFIGS[0], table.getSubTable("Pod 1"))),
            new SwervePod(new OrignalSwervePodIO(LegacyDriveConstants.POD_CONFIGS[1], table.getSubTable("Pod 2"))),
            new SwervePod(new OrignalSwervePodIO(LegacyDriveConstants.POD_CONFIGS[2], table.getSubTable("Pod 3"))),
            new SwervePod(new OrignalSwervePodIO(LegacyDriveConstants.POD_CONFIGS[3], table.getSubTable("Pod 4"))),
    };

    public void set(final ChassisSpeeds speeds) {
        final var states = kinematics.toSwerveModuleStates(speeds);

        for (var i = 0; i < states.length; i++) {
            swervePods[i].set(states[i]);
        }
    }

    public Command fullDrive(final XboxOne inputs) {
        return run(() -> {
            // X and Y are swapped here due to trigonometry
            // Having them match will make the robot think forward is to it's right
            // Having them swapped will make it think forward is properly forward
            final var x = inputs.leftStickY.get();
            final var y = inputs.leftStickX.get();
            final var r = inputs.rightStickX.get();

            final var speeds = new ChassisSpeeds(x, y, Math.toRadians(r));
            set(speeds);
        });
    }
}
