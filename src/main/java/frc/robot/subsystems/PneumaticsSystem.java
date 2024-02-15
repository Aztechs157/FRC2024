// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticsSystem extends SubsystemBase {

    private final Compressor compressor = new Compressor(PneumaticsConstants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH);

    private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsConstants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH, PneumaticsConstants.INTAKE_SOLENOID_EXTEND_CHANNEL,
            PneumaticsConstants.INTAKE_SOLENOID_RETRACT_CHANNEL);

    private final DoubleSolenoid deflectorSolenoid = new DoubleSolenoid(PneumaticsConstants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH, PneumaticsConstants.DEFLECTOR_SOLENOID_EXTEND_CHANNEL,
            PneumaticsConstants.DEFLECTOR_SOLENOID_RETRACT_CHANNEL);

    private final DoubleSolenoid hangerPinSolenoid = new DoubleSolenoid(PneumaticsConstants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH, PneumaticsConstants.HANGER_PIN_SOLENOID_EXTEND_CHANNEL,
            PneumaticsConstants.HANGER_PIN_SOLENOID_RETRACT_CHANNEL);

    private boolean intakeOpen = false;
    private boolean deflectorOpen = false;
    private boolean hangerPinOpen = false;

    /** Creates a new Compressor. */
    public PneumaticsSystem() {
        compressor.enableDigital();
    }

    public void deployIntake(final DoubleSolenoid.Value value) {
        intakeOpen = value == DoubleSolenoid.Value.kForward;
        intakeSolenoid.set(value);
    }

    public void deployDeflector(final DoubleSolenoid.Value value) {
        deflectorOpen = value == DoubleSolenoid.Value.kForward;
        deflectorSolenoid.set(value);
    }

    public void deployHangerPin(final DoubleSolenoid.Value value) {
        hangerPinOpen = value == DoubleSolenoid.Value.kForward;
        hangerPinSolenoid.set(value);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
