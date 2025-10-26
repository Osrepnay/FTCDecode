/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

import java.util.ArrayDeque;

@TeleOp
public class YugePhart extends OpMode {
    private Robot robot;
    private TaskRunner runner;
    private Telemetry dash;
    private InputManager inputManager;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        runner = new TaskRunner();
        dash = FtcDashboard.getInstance().getTelemetry();
        inputManager = new InputManager();
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> robot.drivetrain.isHeadingLocked(),
                () -> gamepad1.rumble(100)
        ));
    }

    private boolean didRumble = false;
    private ArrayDeque<Long> lastMs = new ArrayDeque<>();

    @Override
    public void loop() {
        long time = System.currentTimeMillis();
        lastMs.offerLast(time);
        while (lastMs.size() > 10) {
            lastMs.removeFirst();
        }
        if (lastMs.size() > 1) {
            telemetry.addData("loop ms", (double) (lastMs.getLast() - lastMs.getFirst()) / lastMs.size());
        }

        robot.drivetrain.driveBotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
        if (intaking) {
            robot.intake.setPower(((1 + gamepad1.left_stick_y) * 312 / (435. * 10 / 8) * 96 / 80) + 0.2);
        }
         */
        if (gamepad1.rightBumperWasPressed()) {
            robot.performTransition(runner, Robot.Transfer.RIGHT_BUMPER_START);
        }
        if (gamepad1.leftBumperWasPressed()) {
            robot.performTransition(runner, Robot.Transfer.LEFT_BUMPER_START);
        }
        if (gamepad1.rightBumperWasReleased()) {
            robot.performTransition(runner, Robot.Transfer.RIGHT_BUMPER_END);
        }
        if (gamepad1.crossWasPressed()) {
            robot.toggleUnlockOverride();
        }

        telemetry.addData("state", robot.getState());
        telemetry.addData("rpm", robot.launcher.getCurrentRpm());
        dash.addData("rpm", robot.launcher.getCurrentRpm());
        dash.addData("target rpm", robot.launcher.getTargetRpm());
        telemetry.addData("headingLock", robot.drivetrain.getHeadingLock().map(x -> x * 180 / Math.PI).orElse(0.0));
        telemetry.addData("heading", robot.drivetrain.getHeading() * 180 / Math.PI);
        telemetry.addData("range", robot.camera.getRange());

        dash.update();
        runner.update();
        robot.update();
        inputManager.update();
    }
}