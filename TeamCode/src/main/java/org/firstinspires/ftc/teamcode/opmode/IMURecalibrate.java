package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@TeleOp
public class IMURecalibrate extends OpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    private boolean first = true;

    @Override
    public void loop() {
        if (first) {
            pinpoint.resetPosAndIMU();
            first = false;
        }
        pinpoint.update();

        if (pinpoint.getHeading(AngleUnit.DEGREES) < 0.01
                && pinpoint.getPosX(DistanceUnit.MM) < 0.01
                && pinpoint.getPosY(DistanceUnit.MM) < 0.01) {
            requestOpModeStop();
        }
    }
}
