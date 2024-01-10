package org.assabet.aztechs157;

import org.assabet.aztechs157.numbers.Range;

public final class Sanity {
    private Sanity() {
        throw new UnsupportedOperationException("Expect is a utility class");
    }

    public static class SanityError extends RuntimeException {
        public SanityError(final String message) {
            super(message);
        }
    }

    public static NumberSanity check(final double number) {
        return new NumberSanity(number);
    }

    public static record NumberSanity(double value) {
        public NumberSanity containedWithin(final Range range) {
            if (range.contains(value)) {
                return this;
            } else {
                throw new SanityError(value + " was not contained within " + range);
            }
        }

        public NumberSanity equalTo(final double other) {
            if (value == other) {
                return this;
            } else {
                throw new SanityError(value + " was not equal to " + other);
            }
        }

        public NumberSanity notEqualTo(final double other) {
            if (value != other) {
                return this;
            } else {
                throw new SanityError(value + " was equal to " + other);
            }
        }

        public NumberSanity greaterThan(final double other) {
            if (value > other) {
                return this;
            } else {
                throw new SanityError(value + " was not greater than " + other);
            }
        }

        public NumberSanity lessThan(final double other) {
            if (value < other) {
                return this;
            } else {
                throw new SanityError(value + " was not less than " + other);
            }
        }

        public NumberSanity greaterOrEqual(final double other) {
            if (value >= other) {
                return this;
            } else {
                throw new SanityError(value + " was not greater or equal to " + other);
            }
        }

        public NumberSanity lessOrEqual(final double other) {
            if (value <= other) {
                return this;
            } else {
                throw new SanityError(value + " was not less or equal to " + other);
            }
        }
    }

    public static BooleanSanity check(final boolean value) {
        return new BooleanSanity(value);
    }

    public static record BooleanSanity(boolean value) {
        public BooleanSanity equal(final boolean other) {
            if (value == other) {
                return this;
            } else {
                throw new SanityError(value + " was not equal to " + other);
            }
        }
    }
}
