package org.firstinspires.ftc.teamcode.noncents;

import java.util.Arrays;
import java.util.Comparator;
import java.util.stream.IntStream;

public class Lerp {
    private final double[][] points;

    public Lerp(double[][] points) {
        this.points = Arrays.stream(points).sorted(Comparator.comparing(p -> p[0])).toArray(double[][]::new);
    }

    public Lerp(double[] x, double[] y) {
        this(IntStream.range(0, x.length).mapToObj(i -> new double[] {x[i], y[i]}).toArray(double[][]::new));
    }

    public double interpolate(double x) {
        // i is the index of the rightmost point in the relevant line segment
        int i = 0;
        while (i < points.length && points[i][0] < x) {
            i++;
        }
        i = Math.min(points.length - 1, Math.max(1, i));
        double[] left = points[i - 1];
        double[] right = points[i];
        return left[1] + (right[1] - left[1]) * (x - left[0]) / (right[0] - left[0]);
    }

    // effectively mirrors lerp points across x-axis
    public double interpolateMagnitude(double x) {
        return Math.signum(x) * interpolate(Math.abs(x));
    }
}
