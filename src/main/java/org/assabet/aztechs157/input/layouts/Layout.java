package org.assabet.aztechs157.input.layouts;

import org.assabet.aztechs157.input.values.Axis;
import org.assabet.aztechs157.input.values.Button;

public interface Layout {
    /**
     * Retrieve the {@link Button} associated with a {@link Button.Key}
     *
     * @param key The key a button was assigned to
     * @return The associated button
     */
    public Button button(final Button.Key key);

    /**
     * Retrieve the {@link Axis} associated with a {@link Axis.KeyBase}
     *
     * @param key The key an axis was assigned to
     * @return The associated axis
     */
    public Axis axis(final Axis.Key key);
}
