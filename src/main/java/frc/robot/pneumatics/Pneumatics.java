// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase {
    private final Compressor compressor = new Compressor(PneumaticsConstants.COMPRESSOR_ID, PneumaticsModuleType.REVPH);
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsConstants.COMPRESSOR_ID,
            PneumaticsModuleType.REVPH, PneumaticsConstants.SOLENOID_FORWARD_CHANNEL,
            PneumaticsConstants.SOLENOID_REVERSE_CHANNEL);
    private boolean open = false;

    /** Creates a new Compressor. */
    public Pneumatics() {
        compressor.enableDigital();
    }

    public void set(final DoubleSolenoid.Value value) {
        open = value == DoubleSolenoid.Value.kForward;
        solenoid.set(value);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
