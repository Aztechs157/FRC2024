// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticsSystem extends SubsystemBase {

    private final Compressor compressor = new Compressor(PneumaticsConstants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH);

    private final DoubleSolenoid intakeSolenoid;

    private final DoubleSolenoid deflectorSolenoidLeft;

    private final DoubleSolenoid deflectorSolenoidRight;

    private final DoubleSolenoid hangerPinSolenoid = new DoubleSolenoid(PneumaticsConstants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH, PneumaticsConstants.HANGER_PIN_SOLENOID_EXTEND_CHANNEL,
            PneumaticsConstants.HANGER_PIN_SOLENOID_RETRACT_CHANNEL);

    private boolean intakeOpen = false;
    private boolean deflectorLeftOpen = false;
    private boolean deflectorRightOpen = false;
    private boolean hangerPinOpen = false;

    /** Creates a new Compressor. */
    public PneumaticsSystem(boolean isBeta) {
        compressor.enableDigital();
        if (isBeta) {
            intakeSolenoid = new DoubleSolenoid(PneumaticsConstants.PNEUMATICS_HUB_ID,
                    PneumaticsModuleType.REVPH, PneumaticsConstants.INTAKE_SOLENOID_RETRACT_CHANNEL,
                    PneumaticsConstants.INTAKE_SOLENOID_EXTEND_CHANNEL);

            deflectorSolenoidLeft = new DoubleSolenoid(PneumaticsConstants.PNEUMATICS_HUB_ID,
                    PneumaticsModuleType.REVPH, PneumaticsConstants.DEFLECTOR_LEFT_SOLENOID_EXTEND_CHANNEL,
                    PneumaticsConstants.DEFLECTOR_LEFT_SOLENOID_RETRACT_CHANNEL);

            deflectorSolenoidRight = new DoubleSolenoid(PneumaticsConstants.PNEUMATICS_HUB_ID,
                    PneumaticsModuleType.REVPH, PneumaticsConstants.DEFLECTOR_RIGHT_SOLENOID_EXTEND_CHANNEL,
                    PneumaticsConstants.DEFLECTOR_RIGHT_SOLENOID_RETRACT_CHANNEL);
        } else {
            intakeSolenoid = new DoubleSolenoid(PneumaticsConstants.PNEUMATICS_HUB_ID,
                    PneumaticsModuleType.REVPH, PneumaticsConstants.INTAKE_SOLENOID_EXTEND_CHANNEL,
                    PneumaticsConstants.INTAKE_SOLENOID_RETRACT_CHANNEL);

            deflectorSolenoidLeft = new DoubleSolenoid(PneumaticsConstants.PNEUMATICS_HUB_ID,
                    PneumaticsModuleType.REVPH, PneumaticsConstants.DEFLECTOR_LEFT_SOLENOID_EXTEND_CHANNEL,
                    PneumaticsConstants.DEFLECTOR_LEFT_SOLENOID_RETRACT_CHANNEL);

            deflectorSolenoidRight = new DoubleSolenoid(PneumaticsConstants.PNEUMATICS_HUB_ID,
                    PneumaticsModuleType.REVPH, PneumaticsConstants.DEFLECTOR_RIGHT_SOLENOID_EXTEND_CHANNEL,
                    PneumaticsConstants.DEFLECTOR_RIGHT_SOLENOID_RETRACT_CHANNEL);
        }
    }

    public Command setIntakeFoward() {
        return runOnce(() -> deployIntake(DoubleSolenoid.Value.kForward));
    }

    public Command setIntakeReverse() {
        return runOnce(() -> deployIntake(DoubleSolenoid.Value.kReverse));
    }

    public Command setDeflectorFoward() {
        return runOnce(() -> {
            deployDeflectorLeft(DoubleSolenoid.Value.kForward);
            deployDeflectorRight(DoubleSolenoid.Value.kForward);
        });
    }

    public Command setDeflectorReverse() {
        return runOnce(() -> {
            deployDeflectorLeft(DoubleSolenoid.Value.kReverse);
            deployDeflectorRight(DoubleSolenoid.Value.kReverse);
        });
    }

    public void deployIntake(final DoubleSolenoid.Value value) {
        intakeOpen = value == DoubleSolenoid.Value.kForward;
        intakeSolenoid.set(value);
    }

    public void deployDeflectorLeft(final DoubleSolenoid.Value value) {
        deflectorLeftOpen = value == DoubleSolenoid.Value.kForward;
        deflectorSolenoidLeft.set(value);
    }

    public void deployDeflectorRight(final DoubleSolenoid.Value value) {
        deflectorRightOpen = value == DoubleSolenoid.Value.kForward;
        deflectorSolenoidRight.set(value);
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
