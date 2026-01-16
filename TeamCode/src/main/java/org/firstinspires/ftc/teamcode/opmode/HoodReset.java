package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hood;
import org.firstinspires.ftc.teamcode.noncents.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

@TeleOp
public class HoodReset extends OpMode {
    private Hood hood;
    private TaskRunner runner;

    @Override
    public void init() {
        hood = new Hood(hardwareMap);

        runner = new TaskRunner();
        runner.sendTask(
                hood.doSetPosRaw(Hood.HOOD_MAX + 0.15)
                        .andThen(new DelayTask(2000))
                        .andThen(hood.doSetPos(1))
                        .andThen(new DelayTask(2000))
                        .andThen(hood.doSetPos(0))
        );
    }

    @Override
    public void loop() {
        runner.update();
    }
}
