// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSystem extends SubsystemBase {

    private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private DigitalInput noteSensor = new DigitalInput(IntakeConstants.NOTE_SENSOR_CHANNEL);

    /** Creates a new IntakeSystem. */
    public IntakeSystem() {
        intakeMotor.setIdleMode(IntakeConstants.INTAKE_MOTOR_IDLE_MODE);
        Shuffleboard.getTab("Driver").addBoolean("note sensor", this::checkForNote);
    }

    public void set(double velocity) {
        intakeMotor.set(velocity);
    }

    public boolean checkForNote() {
        return !noteSensor.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
