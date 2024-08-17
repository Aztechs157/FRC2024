// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSystem extends SubsystemBase {
    private CANSparkBase shooterMotorLeft;
    private CANSparkBase shooterMotorRight;
    private boolean isBeta;
    public boolean isShooting = false;
    public double currentLeftMotorSet = 0;
    public double currentRightMotorSet = 0;

    private boolean shootIsRunning = false;

    public boolean getShootIsRunning() {
        return shootIsRunning;
    }

    public void setShootIsRunning(boolean setTrue) {
        shootIsRunning = setTrue;
    }

    /** Creates a new Shooter. */
    public ShooterSystem(boolean isBeta, boolean debugModeOn) {
        this.isBeta = isBeta;
        if (isBeta) {
            shooterMotorLeft = new CANSparkFlex(ShooterConstants.SHOOTER_MOTOR_LEFT_ID,
                    MotorType.kBrushless);
            shooterMotorRight = new CANSparkFlex(ShooterConstants.SHOOTER_MOTOR_RIGHT_ID,
                    MotorType.kBrushless);
        } else {
            shooterMotorLeft = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_LEFT_ID,
                    MotorType.kBrushless);
            shooterMotorRight = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_RIGHT_ID,
                    MotorType.kBrushless);
        }
        shooterMotorLeft.setIdleMode(ShooterConstants.SHOOTER_MOTOR_IDLE_MODE);
        shooterMotorRight.setIdleMode(ShooterConstants.SHOOTER_MOTOR_IDLE_MODE);

        if (debugModeOn) {
            Shuffleboard.getTab("Shooter").addDouble("Left Shooter Wheel", this::getLeftEncoderVelocity);
            Shuffleboard.getTab("Shooter").addDouble("Right Shooter Wheel", this::getRightEncoderVelocity);
        }
    }

    public void setLeftMotor(double velocity) {
        shooterMotorLeft.set(-velocity);
    }

    public void setRightMotor(double velocity) {
        shooterMotorRight.set(velocity);
    }

    public void setShooterMotors(double velocity) {
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
        double PIDval;
        if (isBeta) {
            PIDval = ShooterConstants.BETA_SHOOTER_MOTOR_PID_LEFT.calculate(-currentVelocity, desiredVelocity);
        } else {
            PIDval = ShooterConstants.ALPHA_SHOOTER_MOTOR_PID_LEFT.calculate(-currentVelocity, desiredVelocity);
        }
        return PIDval;
    }

    public double rightMotorPID(double desiredVelocity) {
        double currentVelocity = getRightEncoderVelocity();
        double PIDval;
        if (isBeta) {
            PIDval = ShooterConstants.BETA_SHOOTER_MOTOR_PID_RIGHT.calculate(currentVelocity, desiredVelocity);
        } else {
            PIDval = ShooterConstants.ALPHA_SHOOTER_MOTOR_PID_RIGHT.calculate(-currentVelocity, desiredVelocity);
        }
        return PIDval;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
