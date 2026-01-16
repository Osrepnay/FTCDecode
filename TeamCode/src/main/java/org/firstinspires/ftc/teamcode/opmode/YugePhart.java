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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Color;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

import java.util.ArrayDeque;
import java.util.List;

@TeleOp
public class YugePhart extends OpMode {
    private Robot robot;
    private TaskRunner runner;
    private Telemetry dash;
    private InputManager inputManager;
    private List<LynxModule> hubs;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        runner = new TaskRunner();
        dash = FtcDashboard.getInstance().getTelemetry();
        inputManager = new InputManager();

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(h -> h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        if (!Color.getCurrentColor().isPresent()) {
            telemetry.addLine("WARNING: NO COLOR SET! RUN ASetRed OR ASetBlue");
            telemetry.update();
        }
    }

    private final ArrayDeque<Long> lastMs = new ArrayDeque<>();

    private boolean first = true;

    @Override
    public void loop() {
        if (first) {
            first = false;
            runner.sendTask(robot.doInit());
        }

        hubs.forEach(LynxModule::clearBulkCache);

        long time = System.currentTimeMillis();
        lastMs.offerLast(time);
        while (lastMs.size() > 10) {
            lastMs.removeFirst();
        }
        if (lastMs.size() > 1) {
            telemetry.addData("loop ms", (double) (lastMs.getLast() - lastMs.getFirst()) / lastMs.size());
        }

        if (gamepad1.rightBumperWasPressed()) {
            robot.performTransition(runner, Robot.Transfer.RIGHT_BUMPER_START);
        }
        if (gamepad1.leftBumperWasPressed()) {
            robot.performTransition(runner, Robot.Transfer.LEFT_BUMPER_START);
        }
        if (gamepad1.leftBumperWasReleased()) {
            robot.performTransition(runner, Robot.Transfer.LEFT_BUMPER_END);
        }
        if (gamepad1.rightBumperWasReleased()) {
            robot.performTransition(runner, Robot.Transfer.RIGHT_BUMPER_END);
        }
        if (gamepad1.crossWasPressed()) {
            robot.toggleUnlockOverride();
        }

        if (gamepad2.dpad_down) {
            robot.intake.setPower(-gamepad2.right_stick_y);
        }
        if (gamepad2.dpadDownWasReleased()) {
            robot.intake.setPower(0);
        }
        if (gamepad2.left_bumper && gamepad2.rightBumperWasPressed()) {
            if (robot.isAutoRpmStopped()) {
                robot.enableCamera();
            } else {
                robot.disableCamera();
            }
        }
        if (gamepad2.dpadLeftWasPressed()) {
            robot.launcher.fallbackRpm -= 50;
        } else if (gamepad2.dpadRightWasPressed()) {
            robot.launcher.fallbackRpm += 50;
        }
        if (gamepad2.dpadDownWasPressed()) {
            runner.sendTask(robot.launcher.doSetHood(robot.launcher.getHoodPos() - 0.1));
        } else if (gamepad2.dpadUpWasPressed()) {
            runner.sendTask(robot.launcher.doSetHood(robot.launcher.getHoodPos() + 0.1));
        }

        telemetry.addData("state", robot.getState());
        telemetry.addData("rpm", robot.launcher.getCurrentRpm());
        telemetry.addData("target rpm", robot.launcher.getTargetRpm());
        telemetry.addData("fallback rpm", robot.launcher.fallbackRpm);
        telemetry.addData("hood", robot.launcher.getHoodPos());
        telemetry.addData("dist", robot.getGoalDist());
        telemetry.addData("heading", Math.toDegrees(robot.drivetrain.getHeading()));
        telemetry.addData("camera is disabled?", robot.isAutoRpmStopped());
        telemetry.addData("pinpoint x", robot.pinpoint.getPosition().getX(DistanceUnit.MM));
        telemetry.addData("pinpoint y", robot.pinpoint.getPosition().getY(DistanceUnit.MM));
        telemetry.addData("pinpoint deg", robot.pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("fallback rpm", robot.launcher.fallbackRpm);
        dash.addData("rpm", robot.launcher.getCurrentRpm());
        dash.addData("target rpm", robot.launcher.getTargetRpm());
        dash.addData("heading", robot.drivetrain.getHeading());

        dash.update();
        runner.update();
        robot.drivetrain.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        robot.update();
        inputManager.update();
    }
}