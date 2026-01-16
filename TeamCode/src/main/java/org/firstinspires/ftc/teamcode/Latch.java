package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.noncents.ServoWrapper;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

public class Latch {
    public static final double LATCH_LEFT_CLOSE = 0.8;
    public static final double LATCH_LEFT_OPEN = 0.89;

    public static final long LATCH_DELAY = 2000;

    private final ServoWrapper latchLeft;

    public Latch(ServoImplEx latchLeft) {
        latchLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        this.latchLeft = new ServoWrapper(latchLeft, LATCH_DELAY);
    }

    public Latch(HardwareMap hardwareMap) {
        this(hardwareMap.get(ServoImplEx.class, "latchLeft"));
    }

    public Task doClose() {
        return latchLeft.doSetPosition(LATCH_LEFT_CLOSE);
    }

    public Task doOpen() {
        return latchLeft.doSetPosition(LATCH_LEFT_OPEN);
    }
}