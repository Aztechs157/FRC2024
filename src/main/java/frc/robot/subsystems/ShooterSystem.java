// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.ShooterConstants;

public class ShooterSystem extends SubsystemBase {
    public static final CANSparkMax SHOOTER_MOTOR_LEFT = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_LEFT_ID,
            MotorType.kBrushless);;
    public static final CANSparkMax SHOOTER_MOTOR_RIGHT = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_RIGHT_ID,
            MotorType.kBrushless);

    /** Creates a new Shooter. */
    public ShooterSystem() {
    }

    public void set(double velocity) {
        SHOOTER_MOTOR_LEFT.set(velocity);
        SHOOTER_MOTOR_RIGHT.set(velocity);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
