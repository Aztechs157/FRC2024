package org.assabet.aztechs157.numbers;

import org.assabet.aztechs157.input.values.Axis;

public class Deadzone {
    public final Range deadzone;
    public final Range full;
    public final RangeConverter leftConverter;
    public final RangeConverter rightConverter;

    public static Deadzone forAxis(final Range deadzone) {
        return new Deadzone(deadzone, Axis.kDeviceDefaultRange, 0);
    }

    public Deadzone(final Range deadzone, final Range full, final double fullCenter) {
        this.deadzone = deadzone;
        this.full = full;

        final var leftFull = new Range(full.start(), fullCenter);
        final var leftDeadzone = new Range(full.start(), deadzone.start());
        this.leftConverter = new RangeConverter(leftDeadzone, leftFull);

        final var rightFull = new Range(fullCenter, full.end());
        final var rightDeadzone = new Range(deadzone.end(), full.end());
        this.rightConverter = new RangeConverter(rightDeadzone, rightFull);
    }

    public double apply(final double input) {
        if (deadzone.contains(input)) {
            return 0;
        } else if (leftConverter.inputRange.contains(input)) {
            return leftConverter.convert(input);
        } else if (rightConverter.inputRange.contains(input)) {
            return rightConverter.convert(input);
        }

        throw new Error("Attempted to apply deadzone to input outside of full range "
                + full.start() + " to " + full.end());
    }
}
