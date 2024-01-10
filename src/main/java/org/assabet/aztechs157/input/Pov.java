package org.assabet.aztechs157.input;

import java.util.function.IntSupplier;

import org.assabet.aztechs157.input.values.Axis;
import org.assabet.aztechs157.input.values.Button;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class for getting input from a pov.
 */
public class Pov {
    private final IntSupplier degrees;

    public Pov(final IntSupplier degrees) {
        this.degrees = degrees;
    }

    public static Pov fromDriverStation(final int deviceId, final int povId) {
        return new Pov(() -> DriverStation.getStickPOV(deviceId, povId));
    }

    public int get() {
        return degrees.getAsInt();
    }

    public final Axis value = new Axis(() -> get());

    public final Axis y = new Axis(() -> switch (get()) {
        case UP_LEFT, UP, UP_RIGHT -> 1;
        case LEFT, CENTER, RIGHT -> 0;
        case DOWN_LEFT, DOWN, DOWN_RIGHT -> -1;
        default -> 0;
    });

    public final Axis x = new Axis(() -> switch (get()) {
        case DOWN_RIGHT, RIGHT, UP_RIGHT -> 1;
        case DOWN, CENTER, UP -> 0;
        case DOWN_LEFT, LEFT, UP_LEFT -> -1;
        default -> 0;
    });

    public static final int CENTER = -1;
    public static final int UP = 45 * 0;
    public static final int UP_RIGHT = 45 * 1;
    public static final int RIGHT = 45 * 2;
    public static final int DOWN_RIGHT = 45 * 3;
    public static final int DOWN = 45 * 4;
    public static final int DOWN_LEFT = 45 * 5;
    public static final int LEFT = 45 * 6;
    public static final int UP_LEFT = 45 * 7;

    private Button buttonForValue(final int degrees, final String name) {
        return new Button(() -> get() == degrees);
    }

    public final Button center = buttonForValue(CENTER, "Center");
    public final Button up = buttonForValue(UP, "Up");
    public final Button upRight = buttonForValue(UP_RIGHT, "Up Right");
    public final Button right = buttonForValue(RIGHT, "Right");
    public final Button downRight = buttonForValue(DOWN_RIGHT, "Down Right");
    public final Button down = buttonForValue(DOWN, "Down");
    public final Button downLeft = buttonForValue(DOWN_LEFT, "Down Left");
    public final Button left = buttonForValue(LEFT, "Left");
    public final Button upLeft = buttonForValue(UP_LEFT, "Up Left");
}
