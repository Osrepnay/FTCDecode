package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

public class ActionTaskInterop {
    public static Action taskToAction(Task task) {
        boolean[] first = {true};
        return _telemetryPacket -> {
            if (first[0]) {
                first[0] = false;
                task.init();
            }
            return !task.update();
        };
    }

    public static Task actionToTask(Action action) {
        TelemetryPacket trash = new TelemetryPacket();
        return Task.empty().withUpdate(() -> !action.run(trash));
    }
}
