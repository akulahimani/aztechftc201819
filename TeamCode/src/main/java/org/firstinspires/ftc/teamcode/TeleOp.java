/**
 * Created by neeraja on 11/6/2016.
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop", group="Pushbot")
public class TeleOp extends OpMode {

    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightBack;
    DcMotor motorLeftBack;
    DcMotor cascadeLiftMotor;
//    DcMotor scissorLiftMotor;
    DcMotor ballRotater;
    DcMotor ballPuller;
//    DcMotor spoolMotor;
    Servo ballGrabber;

//     Servo gripperRight;
//      Servo gripperLeft;
//      DcMotor motorElevator;
//   Servo servoRightArm;

    /**
     * Constructor
     */
    public TeleOp() {



    }

    /*
     * Code that runs when the op mode is first enabled.
     */
    @Override
    public void init() {

        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        cascadeLiftMotor = hardwareMap.dcMotor.get("cascadeLiftMotor");
//        scissorLiftMotor = hardwareMap.dcMotor.get("scissorLiftMotor");
        ballRotater = hardwareMap.dcMotor.get("ballRotater");
        ballPuller = hardwareMap.dcMotor.get("ballPuller");
//        spoolMotor = hardwareMap.dcMotor.get("spoolMotor");
        ballGrabber = hardwareMap.servo.get("ballGrabber");

//      gripperRight = hardwareMap.servo.get("gripperRight");//
//      gripperLeft = hardwareMap.servo.get("gripperLeft");//
//        motorElevator = hardwareMap.dcMotor.get("motorElevator");
//        servoRightArm = hardwareMap.servo.get("servoRightArm");//








        //General Descriptions
        //port 1 is gripperleft
        //port 2 is gripperRight
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        cascadeLiftMotor.setDirection(DcMotor.Direction.FORWARD);
//        scissorLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        ballRotater.setDirection(DcMotor.Direction.REVERSE);
        ballPuller.setDirection(DcMotor.Direction.FORWARD);
//        spoolMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    /*
     * This method will be called repeatedly in a loop
     */
    @Override
    public void loop() {

        /*
         * Gamepad 1 controls the drive motors.
         */

        // tank drive
        // y equals -1 means the joystick is pushed all of the way forward.
        //x equals -1 means the joystick 0is pushed all the way to the left.

//        float elevator = -gamepad2.right_stick_y;
//        float launcher = gamepad2.left_stick_y;
        double throttle = -gamepad1.left_stick_y;
        double direction = gamepad1.right_stick_x;
        double cascadeLift = gamepad2.left_stick_y;
//        double scissorLift = gamepad2.right_stick_y;
        double rotate = gamepad2.left_stick_x;
        double pull = gamepad2.right_stick_x;
//        double spoolPower = gamepad2.right_stick_x;
        if(gamepad2.dpad_up == true) {
            ballGrabber.setPosition(0);
        }
        else if(gamepad2.dpad_down == true) {
            ballGrabber.setPosition(1);
        }

       // float elevatorthrottle = -gamepad2.right_stick_y;
      //  float elevatorthrottleReverse = gamepad2.left_stick_y;
        double right = throttle - direction;
        double left = (throttle + direction);


        //gripperRight.scaleRange();


       if (gamepad2.dpad_down) {
//           gripperRight.setPosition(1);
          gripperLeft.setPosition(0);
//
        } else if (gamepad2.dpad_up) {
//           gripperRight.setPosition(0);
            gripperLeft.setPosition(1);
//
//        }







        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        cascadeLift = Range.clip(cascadeLift, -1, 1);
//        scissorLift = Range.clip(scissorLift, -1, 1);
        rotate = Range.clip(rotate, -1, 1);
        pull = Range.clip(pull, -1, 1);
//        spoolPower = Range.clip(spoolPower, -1, 1);
//        brush = Range.clip(brush, -1, 1);
//        elevator = Range.clip(elevator, -1, 1);
//        launcher = Range.clip(launcher, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float) scaleInput(right);
        left = (float) scaleInput(left);


//        brush = (float)scaleInput(brush);
//        elevator = (float)scaleInput(elevator);
//        launcher = (float)scaleInput(launcher);

        // write the values to the motors
//        motorRight.setPower(0.6*right);
            motorRightFront.setPower(0.85 * right);
            motorRightBack.setPower(0.85*right);

//        motorLeft.setPower(0.6*left);
            motorLeftFront.setPower(1 * left);
            motorLeftBack.setPower(1*left);

//            scissorLiftMotor.setPower(1*scissorLift);
            cascadeLiftMotor.setPower(1*cascadeLift);

//            spoolMotor.setPower(1*spoolPower);
            ballRotater.setPower(1*rotate);
            ballPuller.setPower(1*pull);


//        if(direction> 0) {
//            motorRightFront.setPower(1 * right);
//            motorRightBack.setPower(-1*right);
//
////        motorLeft.setPower(0.6*left);
//            motorLeftFront.setPower(-1 * left);
//            motorLeftBack.setPower(1*left);
//        }


//        motorElevator.setPower(.50 * elevatorthrottle);
//        motorElevator.setPower(.50 * elevatorthrottleReverse);






        telemetry.addData("left power", left);
        telemetry.addData("right power", right);
    }




    // telemetry.addData("elevator power", elevatorthrottle);
    // telemetry.addData("servo power", servopower);


//        motorLaunchL.setPower(launcher);
//        motorLaunchR.setPower(-launcher);
//        motorBrush.setPower(brush);
//        motorElevator.setPower(elevator);


    /*
     * Send telemetry data back to driver station. Note that if we are using
     * a legacy NXT-compatible motor controller, then the getPower() method
     * will return a null value. The legacy NXT-compatible motor controllers
     * are currently write only.
     * *telemetry.addData("Text", "*** Robot Data***");
     * *telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
     * *telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
     */





    /*
     * Code that runs when the op mode is first disabled.
     */
    @Override
    public void stop() {

    }

//    public void driveForward() throws InterruptedException {
//        if(gamepad2.a) {
//            motorElevator.setPower(0.5);
//            sleep(2000);
//
//        }
////        motorRight.setPower(Range.clip(power, -1.0, 1.0));
//    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
