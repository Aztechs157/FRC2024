package org.assabet.aztechs157.input.values;

import java.util.function.BooleanSupplier;
import java.util.function.UnaryOperator;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Class for getting input from a button. This class has methods and static
 * methods to modify and compose {@link Button}s into a new
 * {@link Button}.
 */
public class Button {
    public static class Key {
    }

    private final BooleanSupplier value;

    public Button(final BooleanSupplier value) {
        this.value = value;
    }

    public static Button fromDriverStation(final int deviceId, final int buttonId) {
        return new Button(() -> DriverStation.getStickButton(deviceId, buttonId));
    }

    public static Button always(final boolean value) {
        return new Button(() -> value);
    }

    public boolean get() {
        return value.getAsBoolean();
    }

    public Button whenPressed(final Command command) {
        new Trigger(value).onTrue(command);
        return this;
    }

    public Button whileHeld(final Command command) {
        new Trigger(value).whileTrue(command);
        return this;
    }

    public Button map(final UnaryOperator<Boolean> body) {
        return new Button(() -> body.apply(get()));
    }

    public Button tap(final BooleanConsumer body) {
        return map(value -> {
            body.accept(value);
            return value;
        });
    }

    /**
     * Inverts the input; similar to a boolean `!`
     *
     * @return A new inverted input
     */
    public Button inverted() {
        return map(value -> !value);
    }

    /**
     * Checks that all inputs are true; similar to a boolean `&&`
     *
     * @param first The first input
     * @param rest  The rest of the inputs
     * @return A new input that is only true when all of the passed inputs are true
     */
    public static Button all(final Button first, final Button... rest) {
        // The first argument is explicit to prevent being given empty arrays

        return new Button(() -> {
            // Check each input individually
            // As soon as one input is false, return false

            if (first != null && first.get() == false) {
                return false;
            }

            for (final var input : rest) {
                if (input != null && input.get() == false) {
                    return false;
                }
            }

            // All inputs are true at this point, so return true
            return true;
        });
    }

    /**
     * Checks that any input is true; similar to a boolean `||`
     *
     * @param first The first input
     * @param rest  The rest of the inputs
     * @return A new input that is true when any of the passed inputs are true
     */
    public static Button any(final Button first, final Button... rest) {
        // The first argument is explicit to prevent being given empty arrays

        return new Button(() -> {
            // Check each input individually
            // As soon as one input is true, return true

            if (first != null && first.get()) {
                return true;
            }

            for (final var input : rest) {
                if (input != null && input.get()) {
                    return true;
                }
            }

            // All inputs are false at this point, so return false
            return false;
        });

    }
}
