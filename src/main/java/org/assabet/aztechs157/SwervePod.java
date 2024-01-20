package org.assabet.aztechs157;

import org.assabet.aztechs157.numbers.Range;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwervePod {

    public interface SwervePodIO {
        public void setDriveSpeed(final double speed);

        public void setAngleSpeed(final double speed);

        public Rotation2d getAnglePosition();

        public double getDrivePosition();

        public double calculateAnglePid(final double measurement, final double setpoint);

        public default void debugValue(final DebugKey key, final double value) {
        }
    }

    public enum DebugKey {
        InputDriveSpeed, InputAngle, CurrentAngle, InitialDelta, ShortestDelta, AnglePidOutput
    }

    private final SwervePodIO io;

    public SwervePod(final SwervePodIO io) {
        this.io = io;
    }

    public static final Range CIRCLE_RANGE = new Range(0, 360);

    public void set(final SwerveModuleState state) {
        goToAngle(wrapDegrees(state.angle.getDegrees()));
        drive(state.speedMetersPerSecond);
    }

    public void directSet(final double driveSpeed, final double angleSpeed) {
        io.setAngleSpeed(angleSpeed);
        drive(driveSpeed);
    }

    public void stop() {
        io.setAngleSpeed(0);
        drive(0);
    }

    private void drive(double speed) {
        io.debugValue(DebugKey.InputDriveSpeed, speed);
        io.setDriveSpeed(speed);
    }

    private double wrapDegrees(final double degrees) {
        var wrapped = degrees % 360;

        if (wrapped < 0) {
            wrapped += 360;
        }

        Sanity.check(wrapped).containedWithin(CIRCLE_RANGE);
        return wrapped;
    }

    // Next 3 methods are used to find the shortest distance to get to a given angle
    private void goToAngle(final double target) {
        io.debugValue(DebugKey.InputAngle, target);
        io.debugValue(DebugKey.CurrentAngle, io.getAnglePosition().getDegrees());

        final var initialDelta = computeInitialDelta(target);
        io.debugValue(DebugKey.InitialDelta, initialDelta);

        final var shortestDelta = computeShortestDelta(initialDelta);
        io.debugValue(DebugKey.ShortestDelta, shortestDelta);

        final var pidOutput = computeAnglePidOutput(shortestDelta);
        io.debugValue(DebugKey.AnglePidOutput, pidOutput);
        io.setAngleSpeed(pidOutput);
    }

    private double computeInitialDelta(final double target) {
        final double initial = io.getAnglePosition().getDegrees();

        Sanity.check(target).containedWithin(CIRCLE_RANGE);
        Sanity.check(initial).containedWithin(CIRCLE_RANGE);

        var initialDelta = target - initial;
        Sanity.check(initialDelta).greaterOrEqual(-360).lessOrEqual(360);

        if (initialDelta < 0) {
            initialDelta += 360;
        }
        Sanity.check(initialDelta).containedWithin(CIRCLE_RANGE);

        return initialDelta;
    }

    private double computeShortestDelta(final double initialDelta) {
        Sanity.check(initialDelta).containedWithin(CIRCLE_RANGE);

        if (initialDelta < 180) {
            return initialDelta;
        } else {
            return initialDelta - 360;
        }
    }

    private double computeAnglePidOutput(final double shortestDelta) {
        final var degrees = io.getAnglePosition().getDegrees();
        final var pidOutput = io.calculateAnglePid(degrees + shortestDelta, degrees);
        return pidOutput;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(io.getDrivePosition(), io.getAnglePosition());
    }
}
