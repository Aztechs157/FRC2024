// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.inputs;

import org.assabet.aztechs157.input.layouts.Layout;
import org.assabet.aztechs157.input.layouts.MapLayout;

import java.util.function.DoubleSupplier;

import org.assabet.aztechs157.input.layouts.DynamicLayout;
import org.assabet.aztechs157.input.models.XboxOne;
import org.assabet.aztechs157.input.values.Axis;
import org.assabet.aztechs157.input.values.Button;
import org.assabet.aztechs157.numbers.Deadzone;
import org.assabet.aztechs157.numbers.Range;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants.XboxSpeeds;

/** Add your docs here. */
public class Inputs extends DynamicLayout {
    public static final Axis.Key driveSpeedX = new Axis.Key();
    public static final Axis.Key driveSpeedY = new Axis.Key();
    public static final Axis.Key rotateSpeed = new Axis.Key();

    public static final Axis.Key manualShoot = new Axis.Key();
    public static final Button.Key autoShoot = new Button.Key();

    public static final Axis.Key manualSourceIntake = new Axis.Key();
    public static final Button.Key autoSourceIntake = new Button.Key();

    public static Inputs createFromChooser() {
        final SendableChooser<Layout> chooser = new SendableChooser<>();
        chooser.setDefaultOption("demo", doubleXBOXLayout(XboxSpeeds.DEMO));
        chooser.addOption("xbox", doubleXBOXLayout(XboxSpeeds.COMPETITION));
        Shuffleboard.getTab("Driver").add("Layout Choose", chooser);

        return new Inputs(chooser);
    }

    private Inputs(final SendableChooser<Layout> chooser) {
        super(chooser::getSelected);
    }

    private static Layout doubleXBOXLayout(final XboxSpeeds speeds) {
        final var layout = new MapLayout();
        final var driver = new XboxOne(ControllerConstants.DRIVER_CONTROLLER_PORT);
        final var operator = new XboxOne(ControllerConstants.OPERATOR_CONTROLLER_PORT);

        final Deadzone xboxDeadzone = Deadzone.forAxis(new Range(-0.1, 0.1));
        final Rotation2d maxRotationPerSecond = Rotation2d.fromDegrees(130);

        final DoubleSupplier driveSpeed = () -> {
            if (driver.leftStickPress.get()) {
                return speeds.slowDrive();
            }
            return speeds.drive();
        };

        layout.assign(driveSpeedX, driver.leftStickX
                .map(xboxDeadzone::apply)
                .scaledBy(driveSpeed));
        layout.assign(driveSpeedY, driver.leftStickY
                .map(xboxDeadzone::apply)
                .scaledBy(driveSpeed));
        layout.assign(rotateSpeed, driver.rightStickX
                .map(xboxDeadzone::apply)
                .scaledBy(driveSpeed)
                .scaledBy(maxRotationPerSecond.getDegrees()));

        // TODO: make the code prioritize manual over automatic
        layout.assign(manualShoot, operator.rightTriggerHeld);
        layout.assign(autoShoot, operator.rightBumper);

        // TODO: make the code prioritize manual over automatic
        layout.assign(manualSourceIntake, driver.leftTriggerHeld);
        layout.assign(autoSourceIntake, driver.leftBumper);

        return layout;
    }

}
