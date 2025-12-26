package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class FallFest extends OpMode {
    private DcMotorEx turret;
    private DcMotorEx shooter;
    private ServoImplEx hood;

    public static int TURRET_RANGE = 800;
    public static double MAX_TURRET_TPS = TURRET_RANGE * 1;

    public static double SHOOTER_SPEED = 0.45;

    public static double HOOD_START = 0.44;
    public static double HOOD_END = 0.82;
    public static double MAX_HOOD_SPEED = (HOOD_END - HOOD_START) * 0.7;

    private int offset = 0;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        hood = hardwareMap.get(ServoImplEx.class, "hood");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        offset = turret.getCurrentPosition();
        turret.setTargetPosition(offset);
        turret.setPower(0.5);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hood.setDirection(Servo.Direction.REVERSE);
        hood.setPosition(HOOD_START);
    }

    private long lastMs = -1;
    private boolean shooting = false;

    @Override
    public void loop() {
        if (lastMs == -1) {
            lastMs = System.currentTimeMillis();
        }
        long delta = System.currentTimeMillis() - lastMs;
        lastMs = System.currentTimeMillis();
        double secFrac = delta / 1000.0;

        turret.setTargetPosition(offset + Math.min(TURRET_RANGE, Math.max(0, (int) (turret.getTargetPosition() - offset + gamepad1.right_stick_x * MAX_TURRET_TPS * secFrac))));
        hood.setPosition(Math.min(HOOD_END, Math.max(HOOD_START, hood.getPosition() + -gamepad1.left_stick_y * MAX_HOOD_SPEED * secFrac)));

        if (gamepad1.crossWasPressed()) {
            if (shooting) {
                shooter.setPower(0);
            } else {
                shooter.setPower(SHOOTER_SPEED);
            }
            shooting = !shooting;
        }
        telemetry.addData("turret target", turret.getTargetPosition());
        telemetry.addData("turret curr", turret.getCurrentPosition());
        telemetry.addData("sohoter", shooter.getPower());
    }
}
