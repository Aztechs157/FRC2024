// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.PneumaticsSystem;

public class Intake extends Command {

    private IntakeSystem intakeSystem;
    private PneumaticsSystem pneumaticsSystem;

    /** Creates a new Intake. */
    public Intake(final IntakeSystem intakeSystem, final PneumaticsSystem pneumaticsSystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeSystem);
        addRequirements(pneumaticsSystem);

        this.intakeSystem = intakeSystem;
        this.pneumaticsSystem = pneumaticsSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pneumaticsSystem.deployIntake(DoubleSolenoid.Value.kForward);
        intakeSystem.set(-IntakeConstants.INTAKE_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSystem.set(0);
        pneumaticsSystem.deployIntake(DoubleSolenoid.Value.kReverse);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return intakeSystem.checkForNote();
    }
}
