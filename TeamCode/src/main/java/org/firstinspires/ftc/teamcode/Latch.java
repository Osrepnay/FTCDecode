package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.noncents.ServoWrapper;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

public class Latch {
    public static final double LATCH_LEFT_CLOSE = 0.21;
    public static final double LATCH_LEFT_OPEN = 0.34;
    public static final double LATCH_RIGHT_CLOSE = 0.225;
    public static final double LATCH_RIGHT_OPEN = 0.1;

    public static final long LATCH_DELAY = 1300;

    private final ServoWrapper latchLeft, latchRight;

    public Latch(ServoImplEx latchLeft, ServoImplEx latchRight) {
        latchLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        latchRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        this.latchLeft = new ServoWrapper(latchLeft, LATCH_DELAY);
        this.latchRight = new ServoWrapper(latchRight, LATCH_DELAY);
    }

    public Latch(HardwareMap hardwareMap) {
        this(
                hardwareMap.get(ServoImplEx.class, "latchLeft"),
                hardwareMap.get(ServoImplEx.class, "latchRight")
        );
    }

    public Task close() {
        return latchLeft.setPosition(LATCH_LEFT_CLOSE)
                .with(latchRight.setPosition(LATCH_RIGHT_CLOSE));
    }

    public Task open() {
        return latchLeft.setPosition(LATCH_LEFT_OPEN)
                .with(latchRight.setPosition(LATCH_RIGHT_OPEN));
    }
}