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
}
