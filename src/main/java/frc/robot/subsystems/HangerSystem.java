// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangerConstants;

public class HangerSystem extends SubsystemBase {

    private final CANSparkMax hangerMotorLeft = new CANSparkMax(HangerConstants.HANGER_MOTOR_LEFT_ID,
            MotorType.kBrushless);
    private final CANSparkMax hangerMotorRight = new CANSparkMax(HangerConstants.HANGER_MOTOR_RIGHT_ID,
            MotorType.kBrushless);

    /** Creates a new HangerSystem. */
    public HangerSystem() {
    }

    public void setMotors(double velocity) {
        hangerMotorLeft.set(velocity);
        hangerMotorRight.set(velocity);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
