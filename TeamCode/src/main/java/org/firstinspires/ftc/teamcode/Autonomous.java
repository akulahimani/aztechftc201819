package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import static java.lang.Thread.sleep;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous1", group = "Pushbot")
public class Autonomous extends OpMode {

    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    public GoldAlignDetector detector;
    public double encoderCounts = 537.6;
    public double wheelDiameter = 4;
    public double circumference = wheelDiameter*Math.PI;
    public double gearRatio = 1;
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    AngleTest angle = new AngleTest();
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go

        // Loop and update the dashboard

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
        double firstStraightDistance = 43.5;
        double firstTurntoLeft = 30;
        double firstTurntoLeftAngle = 45;
        double turnBacktoCenter = 0;
        double turnBacktoCenterAngle = 45;
        double firstTurntoRight = 0;
        double firstTurntoRightAngle = 45;
        double secondStraightDistance = 15;
        double firstReverseDistance = 15;
        double secondTurntoLeft = 0;
        double secondTurntoLeftAngle = 45;
        double secondTurntoRight = 0;
        double secondTurntoRightAngle = 45;
        double turnBack = 0;
        double turnBackAngle = 60;
        double thirdStraightDistance = 43.5;
        double secondTurnBack = 0;
        double secondTurnBackAngle = 35;
        double fourthStraightDistance = 43.5;
        double thirdTurnBack = 0;
        double thirdTurnBackAngle = 180;
        double fifthStraightDistance = 117.5;

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        driveForwardTargetDistance(firstStraightDistance);
        turnLeftbyDistance(firstTurntoLeft);
//        gyroTurn(0.1, 315);
        ///
        try {
            sleep(5000);
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
            ballGrabber1.setPosition(0.9);
            ballGrabber2.setPosition(0.1);
            ballGrabber1.setPosition(0.2);
            ballGrabber2.setPosition(0.8);
            turnLeftbyDistance(thirdTurnBack);
            driveForwardTargetDistance(fifthStraightDistance);


        }
        else {
            turnRightbyDistance(turnBacktoCenter);

            try {
                sleep(5000);
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
                ballGrabber1.setPosition(0.9);
                ballGrabber2.setPosition(0.1);
                ballGrabber1.setPosition(0.2);
                ballGrabber2.setPosition(0.8);
                turnLeftbyDistance(thirdTurnBack);
                driveForwardTargetDistance(fifthStraightDistance);
            }
            else {
                turnRightbyDistance(firstTurntoRight);

                try {
                    sleep(5000);
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
                    ballGrabber1.setPosition(0.9);
                    ballGrabber2.setPosition(0.1);
                    ballGrabber1.setPosition(0.2);
                    ballGrabber2.setPosition(0.8);
                    turnLeftbyDistance(thirdTurnBack);
                    driveForwardTargetDistance(fifthStraightDistance);
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
        telemetry.addData("First Angle: 0-360", getHeading());
        telemetry.update();

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

        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });


        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });


    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360)%360;
    }

    public void gyroTurn( double speed, double angle) {

        // keep looping while we are still active, and not on heading.
//        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
        while(!onHeading(speed,angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
//            leftDrive.setPower(leftSpeed);
//            rightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
//            while (robotError > 180)  robotError -= 360;
//            while (robotError <= -180) robotError += 360;
        return robotError;
    }

    //        public double getError(double targetAngle){
//            double angleError = 0;
//
//            angleError = (targetAngle - getHeading());
//            angleError -= (360*Math.floor(0.5+((angleError)/360.0)));
//
//            return angleError;
//
//        }



}
