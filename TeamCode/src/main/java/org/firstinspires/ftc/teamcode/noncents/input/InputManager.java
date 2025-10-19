package org.firstinspires.ftc.teamcode.noncents.input;

import java.util.ArrayList;
import java.util.List;

public class InputManager {
    private final List<Trigger> triggers;

    public InputManager() {
        triggers = new ArrayList<>();
    }

    public void addTrigger(Trigger trigger) {
        triggers.add(trigger);
    }

    public void update() {
        for (Trigger trigger : triggers) {
            trigger.update();
        }
    }
}
