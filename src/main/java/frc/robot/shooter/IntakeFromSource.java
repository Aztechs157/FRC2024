// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.inputs.Inputs;

public class IntakeFromSource extends Command {
    private final Shooter shooter;
    private final Inputs inputs;

    /** Creates a new IntakeFromSource. */
    public IntakeFromSource(final Shooter shooter, final Inputs inputs) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        addRequirements(shooter);
        this.inputs = inputs;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (inputs.axis(Inputs.manualSourceIntake).get() > 0) {
            final double velocity = inputs.axis(Inputs.manualSourceIntake).get();
        } else if (inputs.button(Inputs.autoSourceIntake).get()) {
            // calculate the apropreate velocity of the shooter motors based on data from
            // the vision subsystem
        }

        // shooter.set(velocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
