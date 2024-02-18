// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSystem extends SubsystemBase {
    private CANSparkMax shooterMotorLeft = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_LEFT_ID,
            MotorType.kBrushless);;
    private CANSparkMax shooterMotorRight = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_RIGHT_ID,
            MotorType.kBrushless);
    public double currentLeftMotorSet = 0;
    public double currentRightMotorSet = 0;

    /** Creates a new Shooter. */
    public ShooterSystem() {
        shooterMotorLeft.setIdleMode(ShooterConstants.SHOOTER_MOTOR_IDLE_MODE);
        shooterMotorRight.setIdleMode(ShooterConstants.SHOOTER_MOTOR_IDLE_MODE);
        Shuffleboard.getTab("Driver").addDouble("Left Shooter Wheel", this::getLeftEncoderVelocity);
        Shuffleboard.getTab("Driver").addDouble("Right Shooter Wheel", this::getRightEncoderVelocity);
    }

    public void setLeftMotor(double velocity) {
        shooterMotorLeft.set(-velocity);
    }

    public void setRightMotor(double velocity) {
        shooterMotorRight.set(velocity);
    }

    public void setMotors(double velocity) {
        shooterMotorLeft.set(-velocity);
        shooterMotorRight.set(velocity);
    }

    public double getLeftEncoderVelocity() {
        return shooterMotorLeft.getEncoder().getVelocity();
    }

    public double getRightEncoderVelocity() {
        return shooterMotorRight.getEncoder().getVelocity();
    }

    public double leftMotorPID(double desiredVelocity) {
        double currentVelocity = getLeftEncoderVelocity();
        return ShooterConstants.SHOOTER_MOTOR_PID_LEFT.calculate(-currentVelocity, desiredVelocity);
    }

    public double rightMotorPID(double desiredVelocity) {
        double currentVelocity = getRightEncoderVelocity();
        return ShooterConstants.SHOOTER_MOTOR_PID_RIGHT.calculate(currentVelocity, desiredVelocity);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
