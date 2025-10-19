package org.firstinspires.ftc.teamcode.noncents;

public class SlewRateLimiter {
    public double unitsPerSecond;

    public SlewRateLimiter(double unitsPerSecond) {
        this.unitsPerSecond = unitsPerSecond;
    }

    private double last;
    private long lastTime = -1;

    public void reset(double input) {
        last = input;
        lastTime = System.currentTimeMillis();
    }

    public double limit(double desired) {
        if (lastTime == -1) {
            reset(desired);
            return desired;
        } else {
            long time = System.currentTimeMillis();
            long delta = time - lastTime;
            lastTime = time;

            double maxDiff = unitsPerSecond * delta / 1000;
            double clamped = Math.min(last + maxDiff, Math.max(last - maxDiff, desired));
            last = clamped;
            return clamped;
        }
    }
}
