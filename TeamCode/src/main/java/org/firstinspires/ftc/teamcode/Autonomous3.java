package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous1WithoutPark", group = "Pushbot")
public class Autonomous3 extends OpMode {

    public GoldAlignDetector detector;
    public double encoderCounts = 537.6;
    public double wheelDiameter = 4;
    public double circumference = wheelDiameter*Math.PI;
    public double gearRatio = 1;



    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightBack;
    DcMotor motorLeftBack;
    DcMotor cascadeLiftMotor;
    DcMotor ballRotater;
    DcMotor ballPuller;
    Servo ballGrabber1;
    Servo ballGrabber2;


    @Override
    public void init() {
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        cascadeLiftMotor = hardwareMap.dcMotor.get("cascadeLiftMotor");
        ballRotater = hardwareMap.dcMotor.get("ballRotater");
        ballPuller = hardwareMap.dcMotor.get("ballPuller");
        ballGrabber1 = hardwareMap.servo.get("ballGrabber1");
        ballGrabber2 = hardwareMap.servo.get("ballGrabber2");
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        cascadeLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        ballRotater.setDirection(DcMotor.Direction.REVERSE);
        ballPuller.setDirection(DcMotor.Direction.REVERSE);
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 35; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        //detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 1000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

    }

    @Override
    public void start() {
        double firstStraightDistance = 0;
        double firstTurntoLeft = 0;
        double turnBacktoCenter = 0;
        double firstTurntoRight = 0;
        double secondStraightDistance = 0;
        double firstReverseDistance = 0;
        double secondTurntoLeft = 0;
        double secondTurntoRight = 0;
        double turnBack = 0;
        double thirdStraightDistance = 0;
        double secondTurnBack = 0;
        double fourthStraightDistance = 0;

        driveForwardTargetDistance(firstStraightDistance);
        turnLeftbyDistance(firstTurntoLeft);

        try {
            sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if(detector.getAligned() == true) {
            driveForwardTargetDistance(secondStraightDistance);
            driveBackwardTargetDistance(firstReverseDistance);
            turnRightbyDistance(secondTurntoRight);
            turnLeftbyDistance(turnBack);
            driveForwardTargetDistance(thirdStraightDistance);
            turnLeftbyDistance(secondTurnBack);
            driveForwardTargetDistance(fourthStraightDistance);
            ballGrabber1.setPosition(1);
            ballGrabber2.setPosition(0);
            ballGrabber1.setPosition(0);
            ballGrabber2.setPosition(1);
//            turnRightbyDistance(secondTurnBack);
//            driveForwardTargetDistance(fourthStraightDistance);


        }
        else {
            turnRightbyDistance(turnBacktoCenter);

            try {
                sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if(detector.getAligned() == true) {
                driveForwardTargetDistance(secondStraightDistance);
                driveBackwardTargetDistance(firstReverseDistance);
                turnLeftbyDistance(turnBack);
                driveForwardTargetDistance(thirdStraightDistance);
                turnLeftbyDistance(secondTurnBack);
                driveForwardTargetDistance(fourthStraightDistance);
                ballGrabber1.setPosition(1);
                ballGrabber2.setPosition(0);
                ballGrabber1.setPosition(0);
                ballGrabber2.setPosition(1);
//                turnRightbyDistance(secondTurnBack);
//                driveForwardTargetDistance(fourthStraightDistance);
            }
            else {
                turnRightbyDistance(firstTurntoRight);

                try {
                    sleep(2000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if(detector.getAligned() == true) {
                    driveForwardTargetDistance(secondStraightDistance);
                    driveBackwardTargetDistance(firstReverseDistance);
                    turnLeftbyDistance(secondTurntoLeft);
                    turnLeftbyDistance(turnBack);
                    driveForwardTargetDistance(thirdStraightDistance);
                    turnLeftbyDistance(secondTurnBack);
                    driveForwardTargetDistance(fourthStraightDistance);
                    ballGrabber1.setPosition(1);
                    ballGrabber2.setPosition(0);
                    ballGrabber1.setPosition(0);
                    ballGrabber2.setPosition(1);
//                    turnRightbyDistance(secondTurnBack);
//                    driveForwardTargetDistance(fourthStraightDistance);

                }
            }
        }


    }

    @Override
    public void loop() {

        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral
        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.
        telemetry.addData("Left Back Motor Position", motorLeftBack.getCurrentPosition());
        telemetry.addData("Left Front Motor Position", motorLeftFront.getCurrentPosition());
        telemetry.addData("Right Back Motor Position", motorRightBack.getCurrentPosition());
        telemetry.addData("Right Front Motor Position", motorRightFront.getCurrentPosition());
    }

    @Override
    public void stop() {
        detector.disable();
    }

    public void driveForwardTargetDistance(double distance) {

        double rotations = distance/circumference;
        double counts = encoderCounts*rotations*gearRatio;

        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftFront.setTargetPosition((int) counts);
        motorRightFront.setTargetPosition((int) counts);
        motorLeftBack.setTargetPosition((int) counts);
        motorRightBack.setTargetPosition((int) counts);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftFront.setPower(0.5);
        motorRightFront.setPower(0.5);
        motorLeftBack.setPower(0.5);
        motorRightBack.setPower(0.5);
    }

    public void driveBackwardTargetDistance(double distance) {

        double rotations = distance/circumference;
        double counts = encoderCounts*rotations*gearRatio;

        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftFront.setTargetPosition((int) counts);
        motorRightFront.setTargetPosition((int) counts);
        motorLeftBack.setTargetPosition((int) counts);
        motorRightBack.setTargetPosition((int) counts);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftFront.setPower(0.5);
        motorRightFront.setPower(0.5);
        motorLeftBack.setPower(0.5);
        motorRightBack.setPower(0.5);
    }

    public void turnLeftbyDistance(double leftDistance) {

        double rotations = leftDistance/circumference;
        double counts = encoderCounts*rotations*gearRatio;

        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftFront.setTargetPosition((int) counts);
        motorRightFront.setTargetPosition((int) counts);
        motorLeftBack.setTargetPosition((int) counts);
        motorRightBack.setTargetPosition((int) counts);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftFront.setPower(-0.5);
        motorRightFront.setPower(0.5);
        motorLeftBack.setPower(-0.5);
        motorRightBack.setPower(0.5);

    }

    public void turnRightbyDistance(double rightDistance) {

        double rotations = rightDistance/circumference;
        double counts = encoderCounts*rotations*gearRatio;

        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftFront.setTargetPosition((int) counts);
        motorRightFront.setTargetPosition((int) counts);
        motorLeftBack.setTargetPosition((int) counts);
        motorRightBack.setTargetPosition((int) counts);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftFront.setPower(0.5);
        motorRightFront.setPower(-0.5);
        motorLeftBack.setPower(0.5);
        motorRightBack.setPower(-0.5);

    }




}
