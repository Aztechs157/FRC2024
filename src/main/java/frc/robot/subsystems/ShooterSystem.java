// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSystem extends SubsystemBase {
    private CANSparkMax shooterMotorLeft = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_LEFT_ID,
            MotorType.kBrushless);;
    private CANSparkMax shooterMotorRight = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_RIGHT_ID,
            MotorType.kBrushless);

    /** Creates a new Shooter. */
    public ShooterSystem() {
    }

    public void set(double velocity) {
        shooterMotorLeft.set(-velocity);
        shooterMotorRight.set(velocity);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
