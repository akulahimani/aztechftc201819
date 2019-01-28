package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Cascade Lift Test", group="Pushbot")
public class Test3 extends OpMode {

    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor m5;
    Servo s1;
   Servo s2;


    @Override
    public void init() {
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
        m5 = hardwareMap.dcMotor.get("m5");

        s1 = hardwareMap.servo.get("servo1");
        s2 = hardwareMap.servo.get("servo2");

    }

    @Override
    public void loop() {
        telemetry.addData("Motor Position", m5.getCurrentPosition());
        double m5power = -gamepad1.left_stick_y;
//       double m2power = -gamepad1.right_stick_y;

        m5power = Range.clip(m5power, -1, 1);
//        m2power = Range.clip(m2power, -1, 1);

        if(m5power >= 0) {
            m5.setPower(0.75*m5power);
        }

        if(m5power < 0) {
            m5.setPower(0.15*m5power);
        }
//        m2.setPower(m2power);

        if(gamepad1.dpad_up) {
            s1.setPosition(0.9);
            s2.setPosition(0.1);
        }

        if(gamepad1.dpad_down) {
            s1.setPosition(0.25);
            s2.setPosition(0.75);
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
