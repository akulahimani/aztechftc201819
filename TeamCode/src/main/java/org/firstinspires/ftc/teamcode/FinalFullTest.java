package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;

@TeleOp(name="Final Full Test", group="Pushbot")
public class FinalFullTest extends OpMode {

    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor m5;
    Servo servo1;
   Servo servo2;
    DcMotor m2;
    Servo s1;
    Servo s2;


    @Override
    public void init() {
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
        m5 = hardwareMap.dcMotor.get("m5");

        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");

        m2 = hardwareMap.dcMotor.get("m2");
        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {
        telemetry.addData("Motor Position", m5.getCurrentPosition());
        telemetry.addData("Servo 1 Position", servo1.getPosition());
        telemetry.addData("Servo 2 Position", servo2.getPosition());
        double m5power = -gamepad2.left_stick_y;
        double m2power = -gamepad2.right_stick_y;

        double throttle = -gamepad1.left_stick_y;
        double direction = gamepad1.right_stick_x;

        double right = throttle - direction;
        double left = (throttle + direction);

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        motorRightFront.setPower(0.75*0.85 * right);
        motorRightBack.setPower(0.75*0.85*right);

        motorLeftFront.setPower(0.75 * left);
        motorLeftBack.setPower(0.75*left);

//       double m2power = -gamepad1.right_stick_y;

        m5power = Range.clip(m5power, -1, 1);
        m2power = Range.clip(m2power, -1, 1);

        m2.setPower(m2power);


//        m2power = Range.clip(m2power, -1, 1);

        if(m5power >= 0) {
            m5.setPower(0.75*m5power);
        }

        if(m5power < 0) {
            m5.setPower(0.15*m5power);
        }
//        m2.setPower(m2power);

        if(gamepad1.dpad_down) {
            servo1.setPosition(0.9);
            servo2.setPosition(0.1);
        }

        if(gamepad1.dpad_up) {
            servo1.setPosition(0.2);
            servo2.setPosition(0.8);
        }

        if(gamepad2.right_bumper) {
            servo1.setPosition(0.825);
            servo2.setPosition(0.175);
//            servo1.setPosition(0.8);
//            servo2.setPosition(0.2);
        }

        if(gamepad2.left_bumper) {
            servo1.setPosition(0.95);
            servo2.setPosition(0.025);


        }

        if(gamepad2.dpad_up) {
           s1.setPosition(0.35);
           s2.setPosition(0.65);

//            s1.setPosition(1);
//            s2.setPosition(0);
        }

        if(gamepad2.dpad_down) {
            s1.setPosition(0);
            s2.setPosition(1);

//            s1.setPosition(0);
//            s2.setPosition(1);
        }

        if(gamepad1.a) {

            motorRightFront.setPower(0.85*right);
            motorRightBack.setPower(0.85*right);

            motorLeftFront.setPower(left);
            motorLeftBack.setPower(left);

        }


//        if(gamepad1.left_stick_y < 0) {
//            s1.setPosition(1);
//        }
//
//        if(gamepad2.left_stick_y > 0) {
//            s1.setPosition(0);
//        }


//            s1.setPosition(1);
//        s2.setPosition(0);
//    }
//
//        if(gamepad1.dpad_down) {
//        s1.setPosition(0);
//        s2.setPosition(1);
//    }

    }
}
