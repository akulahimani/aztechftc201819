package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp public class Test extends OpMode {

    DcMotor m1;
    DcMotor m2;

    @Override
    public void init() {
        m1 = hardwareMap.dcMotor.get("m1");
//        m2 = hardwareMap.dcMotor.get("m2");

    }

    @Override
    public void loop() {
        double m1power = -gamepad1.left_stick_y;
//        double m2power = gamepad1.right_stick_y;

        m1power = Range.clip(m1power, -1, 1);
//        m2power = Range.clip(m2power, -1, 1);

        m1.setPower(0.25*m1power);
//        m2.setPower(m2power);

    }
}
