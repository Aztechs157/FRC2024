package org.assabet.aztechs157.input.values;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

import org.assabet.aztechs157.numbers.Range;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class for getting input from a axis. This class has methods and static
 * methods to modify and compose {@link Axis}s into a new
 * {@link Axis}.
 */
public class Axis {
    public static class Key {
    }

    public static final Range kDeviceDefaultRange = new Range(-1, 1);

    private final DoubleSupplier value;

    public Axis(final DoubleSupplier value) {
        this.value = value;
    }

    public static Axis fromDriverStation(final int deviceId, final int axisId) {
        return new Axis(() -> DriverStation.getStickAxis(deviceId, axisId));
    }

    public static Axis always(final double value) {
        return new Axis(() -> value);
    }

    public double get() {
        return value.getAsDouble();
    }

    public Axis map(final DoubleUnaryOperator body) {
        return new Axis(() -> body.applyAsDouble(get()));
    }

    public Axis tap(final DoubleConsumer body) {
        return map(value -> {
            body.accept(value);
            return value;
        });
    }

    /**
     * Inverts the input by negating the number's sign
     *
     * @return A new inverted input
     */
    public Axis inverted() {
        return map(value -> -value);
    }

    /**
     * Scale the input with a scalar value.
     *
     * @param scale The value to scale by
     * @return A new input with the scale applied
     */
    public Axis scaledBy(final double scale) {
        return map(value -> value * scale);
    }

    /**
     * Scale the input with another input.
     *
     * @param scale The input to retrieve the scale from
     * @return A new input with the scale applied
     */
    public Axis scaledBy(final DoubleSupplier scale) {
        return map(value -> value * scale.getAsDouble());
    }

    public Axis offsetBy(final double offset) {
        return map(value -> value + offset);
    }

    public Axis offsetBy(final DoubleSupplier offset) {
        return map(value -> value + offset.getAsDouble());
    }

    /**
     * Clamp the input to a number within the provided range.
     *
     * @param range The range to clamp to
     * @return A new input with clamp applied
     */
    public Axis clampTo(final Range range) {
        return map(value -> range.clamp(value));
    }
}
