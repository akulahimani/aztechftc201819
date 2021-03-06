package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Grabbing Balls into Robot Test", group="Pushbot")
public class Test2 extends OpMode {

//    DcMotor m1;
    DcMotor m2;
    Servo s1;
   Servo s2;


    @Override
    public void init() {
//        m1 = hardwareMap.dcMotor.get("m1");
        m2 = hardwareMap.dcMotor.get("m2");
        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
    }

    @Override
    public void loop() {
//        double m1power = -gamepad1.left_stick_y;
      double m2power = -gamepad1.right_stick_y;


//        m1power = Range.clip(m1power, -1, 1);
        m2power = Range.clip(m2power, -1, 1);

//        m1.setPower(m1power);
        m2.setPower(m2power);

        if(gamepad1.dpad_up) {
            s1.setPosition(0.90);
            s2.setPosition(0.10);
        }

        if(gamepad1.dpad_down) {
            s1.setPosition(0.35);
            s2.setPosition(0.65);
        }

        telemetry.addData("s1 position", s1.getPosition());
        telemetry.addData("s2 position", s2.getPosition());



//        s1.setPosition(m1power*0.5+0.5);

//        if(m1power < 0) {
//            s1.setPosition(0);
//        }
//
//        if(m1power > 0) {
//            s1.setPosition(1);
//        }

//        telemetry.addData("Joystick Value", m1power);

//        if(gamepad1.dpad_up) {
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
