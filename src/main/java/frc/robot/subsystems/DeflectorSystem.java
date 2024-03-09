// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class DeflectorSystem extends SubsystemBase {

    private CANSparkFlex deflectorMotor = new CANSparkFlex(ShooterConstants.DEFLECTOR_MOTOR_ID, MotorType.kBrushless);
    private SparkLimitSwitch deflectorExtLim = deflectorMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch deflectorRetLim = deflectorMotor.getReverseLimitSwitch(Type.kNormallyOpen);

    /** Creates a new DeflectorSystem. */
    public DeflectorSystem() {
    }

    public void set(double velocity) {
        deflectorMotor.set(velocity);
    }

    public boolean readExtLimitSwitch() {
        return deflectorExtLim.isPressed();
    }

    public boolean readRetLimitSwitch() {
        return deflectorRetLim.isPressed();
    }

}
