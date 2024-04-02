// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSystem extends SubsystemBase {

    private CANSparkBase leftIntakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_LEFT_ID, MotorType.kBrushless);
    private CANSparkBase rightIntakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_RIGHT_ID,
            MotorType.kBrushless);
    private DigitalInput noteSensor = new DigitalInput(IntakeConstants.NOTE_SENSOR_CHANNEL);
    public boolean hasNote = true;

    /** Creates a new IntakeSystem. */
    public IntakeSystem() {
        leftIntakeMotor.setIdleMode(IntakeConstants.INTAKE_MOTOR_IDLE_MODE);
        rightIntakeMotor.setIdleMode(IntakeConstants.INTAKE_MOTOR_IDLE_MODE);
        Shuffleboard.getTab("Driver").addBoolean("note sensor", this::checkForNote);
    }

    public void set(double velocity) {
        leftIntakeMotor.set(-velocity);
        rightIntakeMotor.set(-velocity);
    }

    public boolean checkForNote() {
        return !noteSensor.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
