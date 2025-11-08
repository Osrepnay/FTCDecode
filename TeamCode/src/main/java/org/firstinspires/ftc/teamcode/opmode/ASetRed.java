package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Color;

@TeleOp
public class ASetRed extends OpMode {

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        Color.setCurrentColor(Color.RED);
        requestOpModeStop();
    }
}
