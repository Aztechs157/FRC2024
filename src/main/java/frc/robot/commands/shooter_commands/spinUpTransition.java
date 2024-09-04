// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VarSystem;

public class spinUpTransition extends Command {

    private VarSystem varSystem;
    private double delay;
    private Timer timer = new Timer();

    /** Creates a new spinUpTransition. */
    public spinUpTransition(final VarSystem varSystem, final double delay) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.varSystem = varSystem;
        this.delay = delay;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        varSystem.setSpinUpTransition(true);
        timer.start();
        System.out.println("transitionStart");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        varSystem.setSpinUpTransition(false);
        System.out.println("transitionEnd");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(delay);
    }

}
