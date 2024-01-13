package org.assabet.aztechs157.input.layouts;

import java.util.HashMap;
import java.util.Map;

import org.assabet.aztechs157.input.values.Axis;
import org.assabet.aztechs157.input.values.Button;

/**
 * A simple structure that stores the mapping between keys and inputs. These can
 * be used with {@link DynamicLayout} to allow hot-swapping of layouts.
 */
public class MapLayout implements Layout {
    private final Map<Button.Key, Button> buttons = new HashMap<>();
    private final Map<Axis.Key, Axis> axes = new HashMap<>();

    /**
     * For this Layout, assign a {@link Button.Key} to a {@link Button}.
     * Calling
     * this method multiple times with the same key will override the previous
     * assignment.
     *
     * @param key    The key to assign with
     * @param button The button being assigned
     */
    public void assign(final Button.Key key, final Button button) {
        buttons.put(key, button);
    }

    /**
     * For this Layout, assign a {@link Axis.KeyBase} to a {@link Axis}. Calling
     * this
     * method multiple times with the same key will override the previous
     * assignment.
     *
     * @param key  The key to assign with
     * @param axis The axis being assigned
     */
    public void assign(final Axis.Key key, final Axis axis) {
        axes.put(key, axis);
    }

    /**
     * Retrieve the {@link Button} associated with a {@link Button.Key}
     *
     * @param key The key a button was assigned to
     * @return The associated button
     */
    public Button button(final Button.Key key) {
        return buttons.get(key);
    }

    /**
     * Retrieve the {@link Axis} associated with a {@link Axis.KeyBase}
     *
     * @param key The key an axis was assigned to
     * @return The associated axis
     */
    public Axis axis(final Axis.Key key) {
        return axes.get(key);
    }
}
