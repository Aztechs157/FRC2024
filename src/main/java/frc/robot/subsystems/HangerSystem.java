// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangerConstants;

public class HangerSystem extends SubsystemBase {

    private final CANSparkBase hangerMotorLeft = new CANSparkMax(HangerConstants.HANGER_MOTOR_LEFT_ID,
            MotorType.kBrushless);
    private final CANSparkBase hangerMotorRight = new CANSparkMax(HangerConstants.HANGER_MOTOR_RIGHT_ID,
            MotorType.kBrushless);
    private final SparkLimitSwitch hangerLeftExtLim = hangerMotorLeft.getForwardLimitSwitch(Type.kNormallyOpen);
    private final SparkLimitSwitch hangerLeftRetLim = hangerMotorLeft.getReverseLimitSwitch(Type.kNormallyOpen);
    private final SparkLimitSwitch hangerRighttExtLim = hangerMotorRight.getForwardLimitSwitch(Type.kNormallyOpen);
    private final SparkLimitSwitch hangerRightRetLim = hangerMotorRight.getReverseLimitSwitch(Type.kNormallyOpen);

    /** Creates a new HangerSystem. */
    public HangerSystem() {
    }

    public void setMotors(double velocity) {
        hangerMotorLeft.set(velocity);
        hangerMotorRight.set(velocity);
    }

    public void setLeftMotor(double velocity) {
        hangerMotorLeft.set(velocity);
    }

    public void setRightMotor(double velocity) {
        hangerMotorRight.set(velocity);
    }

    public boolean readLeftExtLimitSwitch() {
        return hangerLeftExtLim.isPressed();
    }

    public boolean readLeftRetLimitSwitch() {
        return hangerLeftRetLim.isPressed();
    }

    public boolean readRightExtLimitSwitch() {
        return hangerRighttExtLim.isPressed();
    }

    public boolean readRightRetLimitSwitch() {
        return hangerRightRetLim.isPressed();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
