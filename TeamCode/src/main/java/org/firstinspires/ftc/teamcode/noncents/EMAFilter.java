package org.firstinspires.ftc.teamcode.noncents;

// exponential moving averages
public class EMAFilter {
    public double smoothingFactor;

    public EMAFilter(double smoothingFactor) {
        this.smoothingFactor = smoothingFactor;
    }

    private double value = 0;
    private long lastUpdate = -1;
    private double lastSample = 0;

    public double update(double sample) {
        long now = System.currentTimeMillis();
        if (lastUpdate == -1) {
            value = sample;
        } else {
            value = sample * smoothingFactor + value * (1 - smoothingFactor);
            // oroboro.com/irregular-ema
            /*
            long delta = now - lastUpdate;
            double a = delta / smoothingFactor;
            double u = Math.exp(-a);
            double v = (1 - u) / a;
            value = (u * value) + ((v - u) * lastSample) + ((1 - v) * sample);
             */
        }
        lastUpdate = now;
        lastSample = sample;
        return value;
    }
}
