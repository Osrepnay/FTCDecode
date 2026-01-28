package org.firstinspires.ftc.teamcode;

import java.util.Optional;

public enum Color {
    RED, BLUE;
    private static Color currentColor = null;

    public static void setCurrentColor(Color c) {
        currentColor = c;
    }

    public static Optional<Color> getCurrentColor() {
        return Optional.ofNullable(currentColor);
    }

    // red is 1, blue is -1
    public static int currentAsMult() {
        return getCurrentColor().orElse(RED) == RED ? 1 : -1;
    }
}
