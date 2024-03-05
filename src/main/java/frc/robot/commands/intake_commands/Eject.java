// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.cosmetics.PwmLEDs;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class Eject extends Command {

    private final IntakeSystem intakeSystem;
    private final ShooterSystem shooterSystem;
    private final PwmLEDs lightSystem;

    private boolean seenNote = false;
    private Timer timer = new Timer();

    /** Creates a new Eject. */
    public Eject(final IntakeSystem intakeSystem, final ShooterSystem shooterSystem, final PwmLEDs lightSystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeSystem, shooterSystem);

        this.intakeSystem = intakeSystem;
        this.shooterSystem = shooterSystem;
        this.lightSystem = lightSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intakeSystem.set(IntakeConstants.INTAKE_SPEED);
        lightSystem.setSolid(Color.kGreen);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterSystem.currentLeftMotorSet += shooterSystem
                .leftMotorPID(Constants.ShooterConstants.SHOOTER_TARGET_RPM_EJECT);
        shooterSystem.currentRightMotorSet += shooterSystem
                .rightMotorPID(Constants.ShooterConstants.SHOOTER_TARGET_RPM_EJECT);
        shooterSystem.setLeftMotor(shooterSystem.currentLeftMotorSet);
        shooterSystem.setRightMotor(shooterSystem.currentRightMotorSet);

        if (intakeSystem.checkForNote()) {
            seenNote = true;
        }
        if (seenNote && !(intakeSystem.checkForNote())) {
            timer.start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSystem.set(0);

        if (!interrupted) {
            intakeSystem.hasNote = true;
            lightSystem.setSolid(Color.kOrange);
        } else {
            shooterSystem.setLeftMotor(0);
            shooterSystem.setRightMotor(0);
            intakeSystem.hasNote = false;
            lightSystem.setDefault();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(ShooterConstants.SHOOTER_SENSOR_WAIT_TIME);
    }
}
