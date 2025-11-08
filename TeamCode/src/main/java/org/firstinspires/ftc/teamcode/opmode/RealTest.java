package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.noncents.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;

@Config
@TeleOp
public class RealTest extends OpMode {
    Servo servo;
    CRServo crServo;
    DcMotorEx motor;
    InputManager inp;
    Drivetrain dt;
    public static String motorName = "intake";
    String lastMotorName = motorName;
    public static String servoName = "";
    String lastServoName = servoName;
    public static String crServoName = "";
    String lastCrServoName = crServoName;

    Telemetry dash;

    @Override
    public void init() {
        if (!crServoName.isEmpty()) {
            crServo = hardwareMap.get(CRServo.class, crServoName);
        }
        if (!servoName.isEmpty()) {
            servo = hardwareMap.get(ServoImplEx.class, servoName);
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        if (!motorName.isEmpty()) {
            motor = hardwareMap.get(DcMotorEx.class, motorName);
        }
        dt = new Drivetrain(hardwareMap,
                hardwareMap.get(IMU.class, "imu"),
                new CachingVoltageSensor(hardwareMap.voltageSensor.iterator().next()));
        inp = new InputManager();
        inp.addTrigger(new Trigger(Trigger.TriggerType.BEGIN, () -> gamepad1.a, () -> servoPos += 0.005));
        inp.addTrigger(new Trigger(Trigger.TriggerType.BEGIN, () -> gamepad1.b, () -> servoPos -= 0.005));

        dash = FtcDashboard.getInstance().getTelemetry();
    }

    public static double servoPos = 0;
    public static double crServoPos = 0;

    @Override
    public void loop() {
        inp.update();
        dt.driveBotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (!motorName.equals(lastMotorName)) {
            lastMotorName = motorName;
            motor = hardwareMap.get(DcMotorEx.class, motorName);
        }
        if (!servoName.equals(lastServoName)) {
            lastServoName = servoName;
            servo = hardwareMap.get(ServoImplEx.class, servoName);
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
            servoPos = 0;
        }
        if (!crServoName.equals(lastCrServoName)) {
            lastCrServoName = crServoName;
            crServo = hardwareMap.get(CRServo.class, crServoName);
            crServoPos = 0;
        }
        if (servo != null) {
            servo.setPosition(servoPos);
        }
        if (crServo != null) {
            crServo.setPower(crServoPos);
        }
        if (motor != null) {
            motor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("motorPower", motor.getPower());
            telemetry.addData("motorpos", motor.getCurrentPosition());
            telemetry.addData("motor vel", motor.getVelocity() / 145.1 * 60);
        }

        dash.update();
    }
}
