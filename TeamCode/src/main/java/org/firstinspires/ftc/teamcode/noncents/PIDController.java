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
    // how much of the previous iter to use on d's lowpass filter
    // 0: none, 1: all
    public double dLowpass;

    public double p;
    public double i;
    public double d;

    private boolean firstLoop = true;
    private long lastTime = 0;
    private double lastError = 0;
    private double errorAccum = 0;
    private double lastSlope = 0;

    public PIDController(double proportional, double integral, double derivative) {
        this(
                proportional,
                integral,
                derivative,
                Double.POSITIVE_INFINITY,
                1,
                Double.POSITIVE_INFINITY,
                0.0
        );
    }

    public PIDController(
            double proportional,
            double integral,
            double derivative,
            double integralCap,
            double decayFactor,
            double integralRange,
            double dLowpass) {
        this.proportional = proportional;
        this.integral = integral;
        this.derivative = derivative;
        this.integralCap = integralCap;
        this.decayFactor = decayFactor;
        this.integralRange = integralRange;
        this.dLowpass = dLowpass;
    }

    public PIDController withIntegralCap(double cap) {
        return new PIDController(proportional, integral, derivative, cap, decayFactor, integralRange, dLowpass);
    }

    public PIDController withDecayFactor(double decay) {
        return new PIDController(proportional, integral, derivative, integralCap, decay, integralRange, dLowpass);
    }

    public PIDController withIntegralRange(double range) {
        return new PIDController(proportional, integral, derivative, integralCap, decayFactor, range, dLowpass);
    }

    public PIDController withDLowpass(double dLowpass) {
        return new PIDController(proportional, integral, derivative, integralCap, decayFactor, integralRange, dLowpass);
    }

    private boolean lastOutOfRange = true;

    public double update(double setpoint, double value) {
        long time = System.currentTimeMillis();
        double error = setpoint - value;
        if (firstLoop) {
            lastTime = time;
            lastError = error;
        }
        long delta = time - lastTime;
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
        p = proportional * error;

        double slope = (error - lastError) / delta;
        if (Double.isNaN(slope)) {
            slope = lastSlope;
        } else if (firstLoop) {
            lastSlope = slope;
        }
        double filteredSlope = (1 - dLowpass) * slope + dLowpass * lastSlope;
        d = derivative * filteredSlope;
        lastSlope = filteredSlope;

        lastTime = time;
        lastError = error;
        firstLoop = false;
        return p + i + d;
    }

    public void reset() {
        firstLoop = true;
        lastTime = 0;
        lastError = 0;
        errorAccum = 0;
        lastSlope = 0;
    }

    public double getErrorAccum() {
        return errorAccum;
    }
}
