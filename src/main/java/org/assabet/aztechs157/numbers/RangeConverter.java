package org.assabet.aztechs157.numbers;

public class RangeConverter {
    public final Range inputRange;
    public final Range outputRange;
    public final double scaleFactor;

    public RangeConverter(final Range inputRange, final Range outputRange) {
        this.inputRange = inputRange;
        this.outputRange = outputRange;
        this.scaleFactor = outputRange.range() / inputRange.range();
    }

    public double convert(final double inputValue) {
        // Shift to zero based input range
        final var basedInput = inputValue - inputRange.start();

        // Scale the zero based input
        final var scaled = basedInput * scaleFactor;

        // Shift from zero based to output range
        final var outputValue = scaled + outputRange.start();

        return outputValue;
    }
}
