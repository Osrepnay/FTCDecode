package org.firstinspires.ftc.teamcode.noncents;

public class PIDController {
    // would be final, but dashboard
    public double proportional;
    public double integral;
    // TODO smooth derivative calculations?
    public double derivative;
    public double integralCap;
    // what to multiply by every second
    public double decayFactor;
    // range before integral kicks in
    public double integralRange;

    private boolean firstLoop = true;
    private long lastTime = 0;
    private double lastError = 0;
    private double errorAccum = 0;

    public PIDController(double proportional, double integral, double derivative) {
        this(
                proportional,
                integral,
                derivative,
                Double.POSITIVE_INFINITY,
                1,
                Double.POSITIVE_INFINITY
        );
    }

    public PIDController(
            double proportional,
            double integral,
            double derivative,
            double integralCap,
            double decayFactor,
            double integralRange) {
        this.proportional = proportional;
        this.integral = integral;
        this.derivative = derivative;
        this.integralCap = integralCap;
        this.decayFactor = decayFactor;
        this.integralRange = integralRange;
    }

    public PIDController withIntegralCap(double cap) {
        return new PIDController(proportional, integral, derivative, cap, decayFactor, integralRange);
    }

    public PIDController withDecayFactor(double decay) {
        return new PIDController(proportional, integral, derivative, integralCap, decay, integralRange);
    }

    public PIDController withIntegralRange(double range) {
        return new PIDController(proportional, integral, derivative, integralCap, decayFactor, range);
    }

    private boolean lastOutOfRange = true;

    public double update(double setpoint, double value) {
        long time = System.currentTimeMillis();
        double error = setpoint - value;
        if (firstLoop) {
            lastTime = time;
            lastError = error;
            firstLoop = false;
        }
        long delta = time - lastTime;
        double i = 0;
        if (Math.abs(error) < integralRange) {
            if (lastOutOfRange) {
                lastOutOfRange = false;
                errorAccum = 0;
            }
            errorAccum += (error + lastError) / 2 * delta;
            errorAccum = Math.max(-integralCap, Math.min(integralCap, errorAccum));
            errorAccum *= Math.pow(decayFactor, (double) delta / 1000);
            i = integral * errorAccum;
        } else {
            lastOutOfRange = true;
        }
        double p = proportional * error;
        double d = derivative * (error - lastError) / delta;
        if (Double.isNaN(d)) {
            d = 0;
        }
        lastTime = time;
        lastError = error;
        return p + i + d;
    }

    public void reset() {
        firstLoop = true;
        lastTime = 0;
        lastError = 0;
        errorAccum = 0;
    }

    public double getErrorAccum() {
        return errorAccum;
    }
}
