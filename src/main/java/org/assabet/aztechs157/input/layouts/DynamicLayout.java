package org.assabet.aztechs157.input.layouts;

import java.util.function.Supplier;

import org.assabet.aztechs157.input.values.Axis;
import org.assabet.aztechs157.input.values.Button;

/**
 * Object that manages layouts. A layout can be selected from Shuffleboard that
 * can then be used by the robot. It maps the inputs of a
 * {@link DynamicLayout} to the desired functions of the robot.
 */
public class DynamicLayout implements Layout {
    private final Supplier<Layout> layoutSupplier;

    public DynamicLayout(final Supplier<Layout> layoutSupplier) {
        this.layoutSupplier = layoutSupplier;
    }

    public Layout getCurrent() {
        return layoutSupplier.get();
    }

    /**
     * Get a button from the currently selected layout.
     *
     * @param key Which button to retrieve
     * @return A {@link Button} and {@link Button.Key} representing the input
     */
    public Button button(final Button.Key key) {
        return new Button(() -> getCurrent().button(key).get());
    }

    /**
     * Get a axis from the currently selected layout.
     *
     * @param key Which axis to retrieve
     * @return A {@link Axis} representing the input
     */
    public Axis axis(final Axis.Key key) {
        return new Axis(() -> getCurrent().axis(key).get());
    }
}
