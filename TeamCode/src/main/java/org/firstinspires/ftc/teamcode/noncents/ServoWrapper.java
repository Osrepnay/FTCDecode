package org.firstinspires.ftc.teamcode.noncents;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.noncents.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import java.util.Optional;

public class ServoWrapper {
    public final ServoImplEx servo;
    public final long moveDelay;

    private boolean doneMoving = true;
    private double lastChangeTo;
    private boolean hasBeenSet = false;

    public ServoWrapper(ServoImplEx servo, long moveDelay) {
        this.servo = servo;
        this.moveDelay = moveDelay;
    }

    public Task setPosition(double pos) {
        return Task.empty()
                .withOneshot(() -> {
                    doneMoving = false;
                    hasBeenSet = true;
                    servo.setPosition(pos);
                })
                .andThen(Task.defer(() -> new DelayTask((long) (moveDelay * Math.abs(pos - lastChangeTo)))))
                // .andThen(new DelayTask(moveDelay / 10))
                .andThen(Task.empty().withOneshot(() -> {
                    lastChangeTo = pos;
                    doneMoving = true;
                }))
                .withResources(this);
    }

    // returns none if current position is unknown
    public Optional<Double> getPosition() {
        if (!hasBeenSet) {
            return Optional.empty();
        }

        if (doneMoving) {
            return Optional.of(lastChangeTo);
        } else {
            return Optional.empty();
        }
    }
}
