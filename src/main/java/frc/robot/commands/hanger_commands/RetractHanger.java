// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hanger_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HangerConstants;
import frc.robot.cosmetics.PwmLEDs;
import frc.robot.subsystems.HangerSystem;

public class RetractHanger extends Command {

    private final HangerSystem hangerSystem;
    private final PwmLEDs lightSystem;

    /** Creates a new RetractHanger. */
    public RetractHanger(final HangerSystem hangerSystem, final PwmLEDs lightSystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(hangerSystem);

        this.hangerSystem = hangerSystem;
        this.lightSystem = lightSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        hangerSystem.setMotors(HangerConstants.LIFT_RETRACT_SPEED);
        System.out.println("lift retract start");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (hangerSystem.readLeftRetLimitSwitch()) {
            hangerSystem.setLeftMotor(0);
        }
        if (hangerSystem.readRightRetLimitSwitch()) {
            hangerSystem.setRightMotor(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hangerSystem.setLeftMotor(0);
        hangerSystem.setRightMotor(0);
        System.out.println("lift retracted");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return hangerSystem.readLeftRetLimitSwitch() && hangerSystem.readRightRetLimitSwitch();
    }
}
