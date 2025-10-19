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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.noncents.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

import java.util.Arrays;
import java.util.Locale;

@TeleOp
public class FeedforwardTuner extends OpMode {
    private final DcMotor[] motors = new DcMotor[2];
    private Telemetry dash;
    private VoltageSensor voltageSensor;

    private TaskRunner runner;

    @Override
    public void init() {
        motors[0] = hardwareMap.get(DcMotor.class, "launcher0");
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1] = hardwareMap.get(DcMotor.class, "launcher1");
        // ??
        voltageSensor = new CachingVoltageSensor(hardwareMap.voltageSensor.iterator().next());
        dash = FtcDashboard.getInstance().getTelemetry();
        runner = new TaskRunner();

        telemetry.setAutoClear(false);
    }

    private final double powerInc = 0.125;
    private final long accelMs = 2500;
    private final double sampleMs = 500;

    private double currPower = 0;
    // unset at very beginning only
    private long lastChange = -1;
    // unset whenever it's not being tracked
    private long startingTicksCount = -1;

    private boolean stopped = false;
    @Override
    public void loop() {
        if (stopped) return;

        long time = System.currentTimeMillis();
        // most of the time we're busy-looping so this technically makes it slower but
        // it doesnt change anything except maybe make it slightly less precise
        long currentTicksCount = motors[0].getCurrentPosition();
        double voltage = voltageSensor.getVoltage();
        if (lastChange == -1 || time - lastChange > accelMs + sampleMs) {
            lastChange = time;
            if (startingTicksCount == -1) {
                startingTicksCount = currentTicksCount;
            }
            double tps = (double) (currentTicksCount - startingTicksCount) / sampleMs * 1000;
            startingTicksCount = -1;
            double rpm = tps / 103.8 * 60;
            // reverse of how its normally done (we are going from voltage-aware to voltage-blind powers)
            double adjustedPower = currPower / 13 * voltage;
            telemetry.addLine(String.format(Locale.ROOT, "%.4f %.4f", rpm, adjustedPower));

            currPower += powerInc;
            if (currPower > 1) {
                currPower = 0;
                stopped = true;
            }
            Arrays.stream(motors).forEach(m -> m.setPower(currPower));
        } else if (time - lastChange > accelMs && startingTicksCount == -1) {
            startingTicksCount = currentTicksCount;
        }
        runner.update();
    }
}