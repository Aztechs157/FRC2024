package org.assabet.aztechs157;

public class PrintLimiter {
    private final int ticksPerPrint;
    private int currentTicks = 0;

    public PrintLimiter(final int ticksPerPrint) {
        this.ticksPerPrint = ticksPerPrint;
    }

    public PrintLimiter tick() {
        currentTicks++;

        if (currentTicks > ticksPerPrint) {
            currentTicks = 0;
        }

        return this;
    }

    public PrintLimiter println(final String message) {
        if (currentTicks == 0) {
            System.out.println(message);
        }

        return this;
    }

    public PrintLimiter print(final String message) {
        if (currentTicks == 0) {
            System.out.print(message);
        }

        return this;
    }

    public PrintLimiter limitBody(final Runnable body) {
        if (currentTicks == 0) {
            body.run();
        }

        return this;
    }
}
