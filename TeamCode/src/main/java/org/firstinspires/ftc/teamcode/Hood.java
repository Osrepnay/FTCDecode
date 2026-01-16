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
    public final Lerp servoPos = new Lerp(new double[][] {{0, HOOD_MIN}, {1, HOOD_MAX}});
    public final long DELAY = 550;

    private final ServoWrapper hoodLeft;
    private final ServoWrapper hoodRight;

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
        return (hoodLeft.servo.getPosition() - HOOD_MIN) / (HOOD_MAX - HOOD_MIN);
    }

    public Task doSetPosRaw(double pos) {
        return hoodLeft.doSetPosition(pos).with(hoodRight.doSetPosition(pos));
    }

    public Task doSetPos(double frac) {
        double pos = servoPos.interpolate(frac);
        return hoodLeft.doSetPosition(pos).with(hoodRight.doSetPosition(pos));
    }
}
