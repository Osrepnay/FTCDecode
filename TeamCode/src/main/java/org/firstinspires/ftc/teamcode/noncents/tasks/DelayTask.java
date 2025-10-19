package org.firstinspires.ftc.teamcode.noncents.tasks;

import java.util.Collections;
import java.util.Set;

public class DelayTask implements Task {

    public final long ms;

    private long start = -1;

    public DelayTask(long ms) {
        this.ms = ms;
    }

    @Override
    public void init() {}

    @Override
    public boolean update() {
        long time = System.currentTimeMillis();
        if (start == -1) {
            start = time;
        }
        return time - start >= ms;
    }

    @Override
    public boolean isCancellable() {
        return false;
    }

    @Override
    public Set<Object> getResources() {
        return Collections.emptySet();
    }
}
