package org.firstinspires.ftc.teamcode.noncents.input;

import java.util.function.BooleanSupplier;

public class Trigger {
    public enum TriggerType {
        BEGIN,
        CONTINUOUS,
        END
    }

    public final TriggerType type;
    public final BooleanSupplier source;
    public final Runnable action;
    private boolean lastOutput = false;

    public Trigger(TriggerType type, BooleanSupplier source, Runnable action) {
        this.type = type;
        this.source = source;
        this.action = action;
    }

    public void update() {
        boolean output = source.getAsBoolean();
        boolean doAction;
        switch (type) {
            case BEGIN:
                doAction = !lastOutput && output;
                break;
            case CONTINUOUS:
                doAction = output;
                break;
            case END:
                doAction = lastOutput && !output;
                break;
            default:
                throw new IllegalStateException("unhandled type");
        }
        lastOutput = output;
        if (doAction) {
            action.run();
        }
    }
}
