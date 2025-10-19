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
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Latch;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.noncents.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

@TeleOp
public class Phes extends OpMode {
    private Launcher launcher;
    private Latch latch;
    private Intake intake;
    private Telemetry dash;
    private VoltageSensor voltageSensor;

    private TaskRunner runner;

    @Override
    public void init() {
        // ??
        voltageSensor = new CachingVoltageSensor(hardwareMap.voltageSensor.iterator().next());
        launcher = new Launcher(hardwareMap, voltageSensor);
        latch = new Latch(hardwareMap);
        intake = new Intake(hardwareMap);
        dash = FtcDashboard.getInstance().getTelemetry();
        runner = new TaskRunner();
    }

    private double rpmTarget = 0;

    @Override
    public void loop() {
        double power = launcher.update();
        if (gamepad1.triangleWasPressed()) {
            runner.sendTask(latch.close());
        }
        if (gamepad1.circleWasPressed()) {
            runner.sendTask(latch.open());
        }
        intake.setPower(-gamepad1.left_stick_y);
        if (gamepad1.dpadLeftWasPressed()) {
            rpmTarget -= 50;
        } else if (gamepad1.dpadRightWasPressed()) {
            rpmTarget += 50;
        }
        launcher.setTargetRpm(rpmTarget);
        // launcher.setTargetRpm(-gamepad1.right_stick_y * Launcher.LAUNCH_RPM);
        // launcher.setRawPower(-gamepad1.right_stick_y);
        dash.addData("rpm", launcher.getCurrentRpm());
        dash.addData("target rpm", launcher.getTargetRpm());
        dash.addData("power", power);
        dash.update();
        runner.update();
    }
}