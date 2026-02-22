package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.noncents.Lerp;
import org.firstinspires.ftc.teamcode.noncents.ServoWrapper;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

public class Hood {
    public static final double HOOD_MIN = 0.4375;
    public static final double HOOD_MAX = 0.8;
    // estimate?
    // hood plane angle, not exit angle (it's 90 - exit angle)
    public static final double HOOD_MIN_DEG = 24.67;
    public static final double HOOD_MAX_DEG = 51.74;
    public static final Lerp servoLerp = new Lerp(new double[][] {{0, HOOD_MIN}, {1, HOOD_MAX}});
    public static final Lerp servoDegLerp = new Lerp(new double[][] {{0, HOOD_MIN_DEG}, {1, HOOD_MAX_DEG}});
    public final long DELAY = 550;

    private final ServoWrapper hoodLeft;
    private final ServoWrapper hoodRight;
    private double currPos = 0;

    public Hood(ServoImplEx hoodLeft, ServoImplEx hoodRight) {
        hoodRight.setDirection(Servo.Direction.REVERSE);
        hoodLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        hoodRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        this.hoodLeft = new ServoWrapper(hoodLeft, DELAY);
        this.hoodRight = new ServoWrapper(hoodRight, DELAY);
    }

    public Hood(HardwareMap hardwareMap) {
        this(hardwareMap.get(ServoImplEx.class, "hoodLeft"), hardwareMap.get(ServoImplEx.class, "hoodRight"));
    }

    public double getPos() {
        return currPos;
    }

    public static double fracToDeg(double frac) {
        return servoDegLerp.interpolate(frac);
    }

    public Task doSetPosRaw(double pos) {
        return hoodLeft.doSetPosition(pos)
                .with(hoodRight.doSetPosition(pos))
                .with(Task.newWithOneshot(() -> currPos = pos));
    }

    public Task doSetPos(double frac) {
        double pos = servoLerp.interpolate(frac);
        return hoodLeft.doSetPosition(pos)
                .with(hoodRight.doSetPosition(pos))
                .with(Task.newWithOneshot(() -> currPos = pos));
    }

    public void setPos(double frac) {
        double pos = servoLerp.interpolate(frac);
        currPos = pos;
        hoodLeft.servo.setPosition(pos);
        hoodRight.servo.setPosition(pos);
    }
}
