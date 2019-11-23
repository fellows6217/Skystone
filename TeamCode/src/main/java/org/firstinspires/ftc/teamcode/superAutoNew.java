package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.ftccommon.SoundPlayer;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YXZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import android.content.Context;
import android.media.AudioManager;
import android.media.ToneGenerator;


abstract public class superAutoNew extends LinearOpMode {

    //*******************************************************************************************
    //Standard Devices

    //gyro flipped is -1 if the gyro is inverted, otherwise it is 1.
    static final int gyroFlipped = 1;
    final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    final String LABEL_GOLD_MINERAL = "Gold Mineral";
    final String LABEL_SILVER_MINERAL = "Silver Mineral";
    //Drive Motors:  FR = Front Right, FL = Front Left, BR = Back Right, BL = Back Left.
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    //ModernRoboticsI2cRangeSensor rangeSensor;


    //*******************************************************************************************
    //Devices Specific to Challenge

    //DcMotor slideMotor;
    //DcMotor mineralLiftR;
    //DcMotor robotLift;
    //Servo binLeveler;
    //Servo binLifter;
    //Servo servo_U;
    //Servo servo_V;
    /*CRServo outake1;
    CRServo outake2;*/
    DcMotor motorBL;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    //*******************************************************************************************
    //Standard Variables
    ModernRoboticsI2cRangeSensor rangeSensor;
    Orientation angles;
    Acceleration gravity;
    boolean iAmRed;
    boolean iAmBlue = !iAmRed;
    int startingQuadrant;

    //  Sounds
    int soundIndex = 0;
    int soundID = -1;
    boolean soundPlaying = false;
    int ss_alarm = 1;
    int ss_bb8_down = 2;
    int ss_bb8_up = 3;
    int ss_darth_vader = 4;
    int ss_fly_by = 5;
    int ss_mf_fail = 6;
    int ss_laser = 7;
    int ss_laser_burst =8;
    int ss_light_saber = 9;
    int ss_light_saber_long = 10;
    int ss_light_saber_short =11;
    int ss_light_speed = 12;
    int ss_mine = 13;
    int ss_power_up = 14;
    int ss_r2d2_up = 15;
    int ss_roger_roger = 16;
    int ss_siren = 17;
    int ss_wookie = 18;

    //*******************************************************************************************
    //Variables Specific to Challenge
    //RelicRecoveryVuMark[] boxOrder = new RelicRecoveryVuMark[4];
    VuforiaTrackables targetsRoverRuckus;
    List<VuforiaTrackable> allTrackables;
    OpenGLMatrix lastLocation;
    //RobotLocation location = new RobotLocation();
    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;
    private ElapsedTime runtime = new ElapsedTime();

    void setUp() {

        //==================================
        //Standard Setup

        configureGyro();

        mapHardware();

        composeTelemetry();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        //==================================
        //Setup Specific to Challenge

        initVuforia();

        initTfod();



        //==================================
        //All Setup completed, proceed...

        waitForStart();

    }

    void configureGyro() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }


    //*******************************************************************************************
    //Standard Classes and Methods

    void mapHardware() {

        //+++++++++++++++++++++++++++++++++++++++++++++++++
        // Standard Devices

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

        //Drive motors
        /*
         * Wheels: controller 2, motors 0,1,2,3
         */
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        Boolean fullRobot = false;


        //+++++++++++++++++++++++++++++++++++++++++++++++++
        //Devices Specific to Challenge

        /*
        robotLift = hardwareMap.dcMotor.get("robotLift");
        robotLift.setDirection(DcMotor.Direction.FORWARD);
        robotLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (fullRobot) {
            slideMotor = hardwareMap.dcMotor.get("slideMotor");
            slideMotor.setDirection(DcMotor.Direction.FORWARD);
            mineralLiftR = hardwareMap.dcMotor.get("mineralLiftR");
            mineralLiftR.setDirection(DcMotor.Direction.FORWARD);
            robotLift = hardwareMap.dcMotor.get("robotLift");
            robotLift.setDirection(DcMotor.Direction.FORWARD);
            binLeveler = hardwareMap.servo.get("binLeveler");
            binLifter = hardwareMap.servo.get("binLifter");
            servo_U = hardwareMap.servo.get("servo_U");
            servo_V = hardwareMap.servo.get("servo_V");
            /*slide = hardwareMap.servo.get("slide"); //
            outake1 = hardwareMap.crservo.get("outake1");
            outake2 = hardwareMap.crservo.get("outake2");*/
    }

    void composeTelemetry() {

        boolean autoClear = false;
        telemetry.setAutoClear(autoClear);
        telemetry.addLine("starting");
        telemetry.update();

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    void Wait(double WaitTime) {
        runtime.reset();
        while ((runtime.seconds() < WaitTime) && (opModeIsActive())) {
            //Comment this out to avoid it overwriting other telemetry
            //telemetry.addData("5", " %2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
        }
    }

    void sR() {
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }

    public void playSound(int SoundIndex) {

        Context myApp = hardwareMap.appContext;

        String  sounds[] =  {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
                "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
                "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie" };

        // create a sound parameter that holds the desired player parameters.
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;

        // Determine Resource IDs for the sounds you want to play, and make sure it's valid.
        if ((soundID = myApp.getResources().getIdentifier(sounds[soundIndex], "raw", myApp.getPackageName())) != 0) {

            // Signal that the sound is now playing.
            soundPlaying = true;

            // Start playing, and also Create a callback that will clear the playing flag when the sound is complete.
            SoundPlayer.getInstance().startPlaying(myApp, soundID, params, null,
                    new Runnable() {
                        public void run() {
                            soundPlaying = false;
                        }
                    });

        }
    }

    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles != null) {
            double currentHeading = (gyroFlipped) * angles.firstAngle;
            //telemetry.addData("Heading", currentHeading);
            //telemetry.update();
            return currentHeading;
        }
        return -9999;
    }

    public double getxAccel() {
        Acceleration acceleration = imu.getAcceleration();
        if (acceleration != null) {
            double xAccel = acceleration.xAccel;
            //telemetry.clear();
            //telemetry.addData("xAccel", xAccel);
            //telemetry.update();
            return xAccel;

        /*
        gravity = imu.getGravity();
        if (gravity != null) {
            double xAccel = gravity.xAccel;
            telemetry.addData("xAccel", xAccel);
            telemetry.update();
            return xAccel;
            */
        }
        return -9999;
    }

    public double getyAccel() {
        Acceleration acceleration = imu.getAcceleration();
        if (acceleration != null) {
            double yAccel = acceleration.yAccel;
            //telemetry.clear();
            //telemetry.addData("yAccel", yAccel);
            //telemetry.update();
            return yAccel;

        /*
        gravity = imu.getGravity();
        if (gravity != null) {
            double yAccel = gravity.yAccel;
            telemetry.addData("yAccel", yAccel);
            telemetry.update();
            return yAccel;
            */
        }
        return -9999;
    }

    public double gyroToCompass(double Heading) {
        Heading = (Heading + 360) % 360;
        return Heading;
    }

    public double compassToGyro(double Heading) {
        Heading = (Heading - 360) % 360;
        return Heading;
    }

    public double tocm(double meters) {
        double cm = meters * 100;
        return cm;
    }

    public double tomm(double meters) {
        double mm = meters * 1000;
        return mm;
    }

    void pivotRight(double wheelPower) {
        motorFL.setPower(wheelPower);
        motorBL.setPower(wheelPower);
        motorFR.setPower(-wheelPower);
        motorBR.setPower(-wheelPower);
    }

    void pivotLeft(double wheelPower) {
        motorFL.setPower(-wheelPower);
        motorBL.setPower(-wheelPower);
        motorFR.setPower(wheelPower);
        motorBR.setPower(wheelPower);
    }

    void smartTranslateOnHeading(double axisPower, double distToGo, double targetHeading, String axisToUse) {
        double basePosx;
        double basePosy;
        if (axisToUse == "x") {
            basePosx = axisPower;
            basePosy = 0;
        } else {
            basePosx = 0;
            basePosy = axisPower;
        }

        double fudgeFactor = .025f;  //location accuracy +/- in m
        int i = 0;
        double tempAccel = 0;
        double totalAccel = 0;
        double aveAccel = 0;
        double currDist = 0;
        double distDiff = 9999;
        runtime.reset();
        double elapsedTime = 0;

        while (opModeIsActive() && (distDiff > fudgeFactor)) {

            if (axisToUse == "x")
                tempAccel = getxAccel();
            else
                tempAccel = getyAccel();

            if (tempAccel != -999) {

                totalAccel = totalAccel + tempAccel;
                elapsedTime = runtime.seconds();
                i++;
                aveAccel = totalAccel / i;
                currDist = .5 * aveAccel * (elapsedTime * elapsedTime);
                distDiff = Math.abs(Math.abs(distToGo) - Math.abs(currDist));

                //answers are in meters
                //convert to cm
                //currDistx = tocm(currDist);
                //convert to mm
                //currDistx = tomm(currDist);

                telemetry.clear();
                telemetry.addData("Ave Accel", aveAccel);
                telemetry.addData("Elapsed Time", elapsedTime);
                telemetry.addData("currDist", currDist);
                telemetry.update();

                adjustHeading(targetHeading, basePosx, basePosy);

            }
        }

        telemetry.clear();
        telemetry.addData("Ave Accel", aveAccel);
        telemetry.addData("Elapsed Time", elapsedTime);
        telemetry.addData("currDist", currDist);
        telemetry.update();

        sR();
    }

    void smartTranslateSingle(double axisPower, double distToGo, String axisToUse) {
        double posx;
        double posy;
        if (axisToUse == "x") {
            posx = axisPower;
            posy = 0;
        } else {
            posx = 0;
            posy = axisPower;
        }

        double FRBLPower = (posy - posx);
        double FLBRPower = (posy + posx);
        double fudgeFactor = .05f;  //location accuracy +/- in m
        int i = 0;
        double tempAccel = 0;
        double totalAccel = 0;
        double aveAccel = 0;
        double currDist = 0;
        double distDiff = 9999;
        runtime.reset();
        double elapsedTime = 0;

        while (opModeIsActive() && (distDiff > fudgeFactor)) {

            if (axisToUse == "x")
                tempAccel = getxAccel();
            else
                tempAccel = getyAccel();

            if (tempAccel != -999) {

                totalAccel = totalAccel + tempAccel;
                elapsedTime = runtime.seconds();
                i++;
                aveAccel = totalAccel / i;
                currDist = .5 * aveAccel * (elapsedTime * elapsedTime);
                distDiff = Math.abs(Math.abs(distToGo) - Math.abs(currDist));

                //answers are in meters
                //convert to cm
                //currDistx = tocm(currDist);
                //convert to mm
                //currDistx = tomm(currDist);

                telemetry.clear();
                telemetry.addData("Ave Accel", aveAccel);
                telemetry.addData("Elapsed Time", elapsedTime);
                telemetry.addData("currDist", currDist);
                telemetry.update();

                motorFR.setPower(FRBLPower);
                motorFL.setPower(FLBRPower);
                motorBR.setPower(FLBRPower);
                motorBL.setPower(FRBLPower);

            }
        }

        telemetry.clear();
        telemetry.addData("Ave Accel", aveAccel);
        telemetry.addData("Elapsed Time", elapsedTime);
        telemetry.addData("currDist", currDist);
        telemetry.update();

        sR();
    }

    void smartTranslate(double posx, double posy, double distx, double disty) {
        double FRBLPower = (posy - posx);
        double FLBRPower = (posy + posx);
        double fudgeFactor = .05f;  //location accuracy +/- in m, cm, or mm
        int i = 0;
        double tempxAccel = 0;
        double totalxAccel = 0;
        double avexAccel = 0;
        double currDistx = 0;
        double xDiff = 9999;
        double tempyAccel = 0;
        double totalyAccel = 0;
        double aveyAccel = 0;
        double currDisty = 0;
        double yDiff = 9999;
        runtime.reset();
        double elapsedTime = 0;

        while ((opModeIsActive()) && ((xDiff > fudgeFactor) || (yDiff > fudgeFactor))) {

            tempxAccel = getxAccel();
            tempyAccel = getyAccel();

            if ((tempxAccel != -999) && (tempyAccel != -9999)) {

                totalxAccel = totalxAccel + tempxAccel;
                totalyAccel = totalyAccel + tempyAccel;
                elapsedTime = runtime.seconds();
                i++;
                avexAccel = totalxAccel / i;
                aveyAccel = totalyAccel / i;
                currDistx = .5 * avexAccel * (elapsedTime * elapsedTime);
                currDisty = .5 * aveyAccel * (elapsedTime * elapsedTime);
                xDiff = Math.abs(Math.abs(distx) - Math.abs(currDistx));
                yDiff = Math.abs(Math.abs(disty) - Math.abs(currDisty));

                //answers are in meters
                //convert to cm
                //currDistx = tocm(currDistx);
                //currDisty = tocm(currDisty);
                //convert to mm
                //currDistx = tomm(currDistx);
                //currDisty = tomm(currDisty);

                telemetry.clear();
                telemetry.addData("Ave xAccel", avexAccel);
                telemetry.addData("Ave yAccel", aveyAccel);
                telemetry.addData("Elapsed Time", elapsedTime);
                telemetry.addData("currDistx", currDistx);
                telemetry.addData("currDisty", currDisty);
                telemetry.update();

                motorFR.setPower(FRBLPower);
                motorFL.setPower(FLBRPower);
                motorBR.setPower(FLBRPower);
                motorBL.setPower(FRBLPower);

            }
        }

        telemetry.clear();
        telemetry.addData("xAccel", tempxAccel);
        telemetry.addData("yAccel", tempyAccel);
        telemetry.addData("Elapsed Time", elapsedTime);
        telemetry.addData("currDistx", currDistx);
        telemetry.addData("currDisty", currDisty);
        telemetry.update();

        sR();
    }

    void smartGyroPivot(double target) {
        //Provide target as a gyro heading (180 [to the left] to -179 [to the right])

        //First set up variables

        float fudgeFactor = .5f;   //heading accuracy +/- in degrees
        double reallyClose = 3;     //distance from desired heading in degrees
        double gettingClose = 10;   //distance from desired heading in degrees
        double maxWheelPower = .5;
        double midWheelPower = .35;
        double minWheelPower = .25;
        double wheelPower = maxWheelPower;
        int turnDirection;
        String turnDirectionString;
        double currentHeading = -9999;
        double distance = 180;
        pivotStates currentPivotState = pivotStates.Init;

        while (opModeIsActive() && (Math.abs(distance) >= fudgeFactor)) {

            currentHeading = getHeading();

            if (currentHeading != 9999) {

                //Determine the default direction of the turn
                distance = target - currentHeading;
                if (distance < 0)
                    //we'd be turning to the right
                    turnDirection = 1;
                else
                    //we'd be turning to the left
                    turnDirection = -1;

                //Calculate the distance in the opposite direction, and select the shortest distance
                double oppositeDistance = 360 - Math.abs(distance);
                distance = Math.min(Math.abs(distance), Math.abs(oppositeDistance));

                if (distance == oppositeDistance)
                    //if the shorter distance is in the opposite direction, change the turn direction
                    turnDirection = turnDirection * -1;

                if (turnDirection > 0) turnDirectionString = "Right";
                else turnDirectionString = "Left";

                /*  Determine if we're crossing 180, using compass headings to test:
                    if we’re turning to the right and (180 < current) AND (180 > target),
                    or
                    if we’re turning left and (180 > current) AND (180 < target),
                    then use compass headings */
                double compassTarget = gyroToCompass(target);
                double compassHeading = gyroToCompass(currentHeading);
                double compassDistance = Math.abs(compassTarget - compassHeading);
                if (((turnDirection == 1) && (compassHeading < 180) && (compassTarget > 180)) || ((turnDirection == -1) && (compassHeading > 180) && (compassTarget < 180) && (compassDistance < 180))) {
                    currentHeading = compassHeading;
                    target = compassTarget;
                }

                //set wheel power based on how close we are to the target
                if (distance < reallyClose)         //we're really close
                    wheelPower = minWheelPower;     //so go very slowly
                else if (distance < gettingClose)   //we're getting close
                    wheelPower = midWheelPower;     //so slow down a bit
                else                                //we're a long way away, still
                    wheelPower = maxWheelPower;     //punch it!

                if (Math.abs(distance) <= fudgeFactor)
                    currentPivotState = pivotStates.None;   //we're inside the fudge factor, so stop
                else if (turnDirection < 0)
                    currentPivotState = pivotStates.Left;
                else
                    currentPivotState = pivotStates.Right;


                telemetry.clearAll();
                telemetry.addData("Current Heading: ", currentHeading);
                telemetry.addData("Target: ", target);
                telemetry.addData("Distance", distance);
                telemetry.addData("Wheel Power", wheelPower);
                telemetry.addData("Turn Direction", turnDirectionString);
                telemetry.addData("Pivot State", currentPivotState);
                telemetry.update();

                switch (currentPivotState) {

                    case Right:
                        pivotRight(wheelPower);
                        break;

                    case Left:
                        pivotLeft(wheelPower);
                        break;

                    case None:
                        sR();
                        break;

                }

            }
        }

        sR();
        ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
        toneG.startTone(ToneGenerator.TONE_CDMA_ALERT_CALL_GUARD, 200);
    }

    void followHeading(int targetHeading, double time, float basePosx, float basePosy) {
        runtime.reset();
        while (((runtime.seconds() < time) && (opModeIsActive()))) {
            adjustHeading(targetHeading, basePosx, basePosy);
        }
        sR();
    }

    void adjustHeading(double targetHeading, double basePosx, double basePosy) {
        double currentHeading = getHeading();
        //telemetry.addData("First Angle", angles.firstAngle);
        double addPower = (targetHeading - currentHeading) * .025;
        //telemetry.addData("add power", addPower);

        double FRBLPower = basePosy - basePosx;
        double FLBRPower = basePosy + basePosx;
        motorFR.setPower(FRBLPower + addPower);
        motorFL.setPower(FLBRPower - addPower);
        motorBR.setPower(FLBRPower + addPower);
        motorBL.setPower(FRBLPower - addPower);
    }

    void translate(double posx, double posy, double seconds, double power) {
        double FRBLPower = (posy - posx) * power;
        double FLBRPower = (posy + posx) * power;
        motorFR.setPower(FRBLPower);
        motorFL.setPower(FLBRPower);
        motorBR.setPower(FLBRPower);
        motorBL.setPower(FRBLPower);
        Wait(seconds);
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
    }

    void translateForever(double posx, double posy, double power) {
        double FRBLPower = (posy - posx) * power;
        double FLBRPower = (posy + posx) * power;
        motorFR.setPower(FRBLPower);
        motorFL.setPower(FLBRPower);
        motorBR.setPower(FLBRPower);
        motorBL.setPower(FRBLPower);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /*
    void pivotTo(int target) {
        //Pivot to counterclockwise is positive.
        //Pivot to clockwise is negative.
        float fudgeFactor = .5f;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;
        double wheelPower = .2;
        telemetry.addData("target: ", target);
        telemetry.update();

        while ((currentHeading < (target - fudgeFactor)) || (currentHeading > (target + fudgeFactor))) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (angles != null) {
                currentHeading = angles.firstAngle;
                telemetry.addData("current heading: ", currentHeading);
                telemetry.update();
                if (target - currentHeading > 0) {
                    motorFL.setPower(-wheelPower);
                    motorBL.setPower(-wheelPower);
                    motorFR.setPower(wheelPower);
                    motorBR.setPower(wheelPower);
                } else {
                    motorFL.setPower(wheelPower);
                    motorBL.setPower(wheelPower);
                    motorFR.setPower(-wheelPower);
                    motorBR.setPower(-wheelPower);
                }
            }

        }
        sR();
    }

    public double compassConverter(double raw) {
        double compass;

        if (raw < 0) {
            compass = 360 + raw;
        } else {
            compass = raw;
        }
        return compass;
    }
    */

    /*
    void getQuadrant() {

        updateLocation();

        double x = location.getX();
        double y = location.getY();

        if (x > 0 && y > 0)
            startingQuadrant = 1;
        else if (x < 0 && y > 0)
            startingQuadrant = 2;
        else if (x < 0 && y < 0)
            startingQuadrant = 3;
        else
            startingQuadrant = 4;
    }
*/


    void configVuforiaSkystone() {

        // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
        final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
        final boolean PHONE_IS_PORTRAIT = false  ;

        final String VUFORIA_KEY = "AYhHUgX/////AAABmTO0g2PsdUqpg5xo" +
                "96O7OkOB7qrwOjE24wV71lIm/MF9g96awd677rj7LrgQKUJAewgWkAAxn1MUJtUyiq" +
                "9iesjKF+QNXlKr5qCAb69hI268sYjjCJ+PqVBtMrlcIG1F4l2osl9zIk9tYAYfLXKl" +
                "T351h1yRW1AqAdHJaHwt861ztrh4EW/1WjOV3/yT4SDtrJivhfmU0c51IqPUEJ0xqbWFr2saxvS/cSkH4e+hFIImM/jIw5JkaizeznuFTA" +
                "TnWTq9Spp/EhPPaQXJtScNP3DDaNDdfiqT9opwsxuQlEe1YF19sHjtenGD7/PcUlQXVS+VKbxaSqd/Cnq4+/3bOuhNzFoTVbBKZ16DQ9EZeCJM";

        // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here
        final float mmPerInch        = 25.4f;
        final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

        // Constant for Stone Target
        final float stoneZ = 2.00f * mmPerInch;

        // Constants for the center support targets
        final float bridgeZ = 6.42f * mmPerInch;
        final float bridgeY = 23 * mmPerInch;
        final float bridgeX = 5.18f * mmPerInch;
        final float bridgeRotY = 59;                                 // Units are degrees
        final float bridgeRotZ = 180;

        // Constants for perimeter targets
        final float halfField = 72 * mmPerInch;
        final float quadField  = 36 * mmPerInch;

        /**
         * This is the webcam we are to use. As with other hardware devices such as motors and
         * servos, this device is identified using the robot configuration tool in the FTC application.
         */
        WebcamName webcamName = null;

        boolean targetVisible = false;
        float phoneXRotate    = 0;
        float phoneYRotate    = 0;
        float phoneZRotate    = 0;

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsSkyStone.activate();

    }


    /**
     * Initialize the Vuforia localization engine.
     */
    void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AYhHUgX/////AAABmTO0g2PsdUqpg5xo" +
                "96O7OkOB7qrwOjE24wV71lIm/MF9g96awd677rj7LrgQKUJAewgWkAAxn1MUJtUyiq" +
                "9iesjKF+QNXlKr5qCAb69hI268sYjjCJ+PqVBtMrlcIG1F4l2osl9zIk9tYAYfLXKl" +
                "T351h1yRW1AqAdHJaHwt861ztrh4EW/1WjOV3/yT4SDtrJivhfmU0c51IqPUEJ0xqbWFr2saxvS/cSkH4e+hFIImM/jIw5JkaizeznuFTA" +
                "TnWTq9Spp/EhPPaQXJtScNP3DDaNDdfiqT9opwsxuQlEe1YF19sHjtenGD7/PcUlQXVS+VKbxaSqd/Cnq4+/3bOuhNzFoTVbBKZ16DQ9EZeCJM";

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    void initTfod() {
        final String TFOD_MODEL_ASSET = "Skystone.tflite";
        final String LABEL_FIRST_ELEMENT = "Stone";
        final String LABEL_SECOND_ELEMENT = "Skystone";



        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    //Write convert back from compass

    void distCorrector(double trgDistance) {

        double curDistance = rangeSensor.getDistance(DistanceUnit.CM);
        double fstDistance = curDistance - trgDistance;

        boolean go = true;
        int i = 0;

        // While not our state do ...
        while (go) {

            curDistance = rangeSensor.getDistance(DistanceUnit.CM);
            double chgDistance = curDistance - trgDistance;
            double direction;
            if (chgDistance > 0) {
                direction = 1;
            } else {
                direction = -1;
            }

            if (Math.abs(chgDistance) >= 2) {

                // Slow down
                double adjust = 0.20 * direction;

                //(Math.abs(chgDistance / fstDistance)) * direction;
                //if (adjust > 0.25) { adjust = 0.25; }

                telemetry.addData("current distance", curDistance);
                telemetry.addData("target distance", trgDistance);
                telemetry.addData("direction", direction);
                telemetry.addData("adjust", adjust);
                telemetry.addData("i", i);
                telemetry.update();

                // Power the motors
                motorFL.setPower(adjust);
                motorFR.setPower(adjust);
                motorBL.setPower(adjust);
                motorBR.setPower(adjust);
            } else {
                go = false;
            }
            i++;
            //    if (i > 30) {
            //        go = false;
            //    }
        }
        sR();
    }


    //*******************************************************************************************
    //Classes Specific to Challenge

    void tensorFlowTest() {
        tfod.activate();
        runtime.reset();
        telemetry.setAutoClear(true);
        translateForever(1, 0, 0.7);
        while (true) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() >= 1) {
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                telemetry.addData("Gold Mineral Position", "Visible");
                                sR();
                                double timePast = runtime.seconds();
                                if (3.8 - timePast > 0.1) {
                                    followHeading(0, 3.8 - timePast, 1, 0);
                                }
                                telemetry.addData("Seconds Past: ", timePast);
                                telemetry.update();
                                tfod.deactivate();
                                return;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Not Visible");
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }
    }

    /*
    void updateLocation() {
        while (opModeIsActive()) {
            // check all the trackable target to see which one (if any) is visible.
            Boolean targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    //telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                //telemetry.addData("Pos (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                //        translation.get(0), translation.get(1), translation.get(2));

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                location.setPositionValues(translation.get(0), translation.get(1), rotation.thirdAngle);
                break;
            } else if (!targetVisible) {
                // Eliminate search algorithm temporarily
            } else {
                configureGyro();
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double currentHeading = angles.firstAngle;
                int degrees = (int) currentHeading;
                while (targetVisible == false) {
                    degrees += 13;
                    if (degrees > 180)//This is incorrect since there is no 360. Range is from 0->180, -180->0
                        break;
                    pivotTo(degrees);
                    Wait(2);
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                            telemetry.addData("Visible Target", trackable.getName());
                            targetVisible = true;
                            break;
                        }
                    }
                    if (targetVisible)
                        break;
                }
                if (!targetVisible) {
                    telemetry.addData("We have pivoted 360 degrees and haven't seen anything.  We Have Depression Now!  Target Visible = ", targetVisible);
                    sR();
                    //Move Code: eventually use range sensor to move NOT INTO A CORNER
                    //This stop robot is because we have pivoted 360 degrees but still havent seen a picture
                    //We will have to work on this but for now, whatever :)
                }
            }
            telemetry.update();
        }
    }

    void goToPoint(double DestinationX, double DestinationY) {
        telemetry.addData("goToPoint destX,Y:",
                "%f %f", DestinationX, DestinationY);

        double CurrentX, CurrentY, X, Y;//X and Y JOYstick coordinates

        boolean go = true;

        // While not at desired location
        while (go) {
            updateLocation();
            CurrentX = location.getX(); //Method to Read current location
            CurrentY = location.getY();
            X = (DestinationX - CurrentX);
            Y = (DestinationY - CurrentY);
            telemetry.addData("Diff:  ", "%f %f", X, Y);
            double divBy = Math.max(Math.abs(X), Math.abs(Y));
            double chgDistance = Math.sqrt((X * X) + (Y * Y));
            X = X / divBy;
            Y = Y / divBy;
            telemetry.addData("  Power: ", "%f %f", X, Y);

            if (Math.abs(chgDistance) >= 300) {
                //double posx = Vuforia_JoystickX(X,Y, Theta);
                //double posy = Vuforia_JoystickY(X,Y, Theta);
                double posx = X;
                double posy = Y;
                telemetry.addData("  Pos  :", "%f %f", CurrentX, CurrentY);
                telemetry.addData("  Chg  :", "%f", chgDistance);
                telemetry.update();
                Wait(.25);

                //Power the motors
                if ((posy != 0) || (posx != 0)) {
                    double FRBLPower = ((-posy) - posx) * 0.3;
                    double FLBRPower = ((-posy) + posx) * 0.3;
                    motorFR.setPower(FRBLPower);
                    motorFL.setPower(FLBRPower);
                    motorBR.setPower(FLBRPower);
                    motorBL.setPower(FRBLPower);
                }
            } else {
                motorFR.setPower(0);
                motorFL.setPower(0);
                motorBR.setPower(0);
                motorBL.setPower(0);
                go = false;
            }
        }
        sR();
    }

    public double Vuforia_JoystickX(double X, double Y, double Theta) {
        double JoystickX = (X * Math.cos(Theta)) - (Y * Math.sin(Theta));
        telemetry.addData("JoystickX", "%f", JoystickX);
        return JoystickX;
    }

    public double Vuforia_JoystickY(double X, double Y, double Theta) {
        double JoystickY = (X * Math.sin(Theta)) + (Y * Math.cos(Theta));
        telemetry.addData("JoystickY", "%f", JoystickY);
        return JoystickY;

    }

    void pivotToVuforia(int target) {
        //Pivot to counterclockwise is positive.
        //Pivot to clockwise is negative.
        float fudgeFactor = .5f;
        updateLocation();
        double currentHeading = location.getHeading();
        double wheelPower = .2;
        telemetry.addData("pivotToVuforia target: ", target);
        telemetry.update();

        while ((currentHeading < (target - fudgeFactor)) || (currentHeading > (target + fudgeFactor))) {
            updateLocation();
            currentHeading = location.getHeading();
            //telemetry.addData("current heading: ", currentHeading);
            //telemetry.update();
            if (target - currentHeading > 0) {
                motorFL.setPower(-wheelPower);
                motorBL.setPower(-wheelPower);
                motorFR.setPower(wheelPower);
                motorBR.setPower(wheelPower);
            } else {
                motorFL.setPower(wheelPower);
                motorBL.setPower(wheelPower);
                motorFR.setPower(-wheelPower);
                motorBR.setPower(-wheelPower);
            }
        }
        telemetry.addData("  Final heading: ", currentHeading);
        telemetry.update();
        sR();
    }
*/

    void tensorFlowCase() {
        states currentState = states.Space;
        tfod.activate();
        //telemetry.setAutoClear(true);
        int counter = 0;
        List<Recognition> updatedRecognitions;

        while (true) {
            adjustHeading(0, 0.5f, 0);
            if (tfod != null) {
                updatedRecognitions = tfod.getUpdatedRecognitions();

                switch (currentState) {
                    case Space:
                        if (updatedRecognitions != null) {
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    counter++;
                                    currentState = states.Gold;
                                    telemetry.addData("Counter ", counter);
                                    telemetry.addData("State: ", "Gold");
                                    telemetry.update();

                                    //Robot Functions
                                    //sR();
                                    //followHeading(0, 0.1, 0, 1);
                                    //followHeading(0, 0.1, 0, -1);
                                    //adjustHeading(0, 0.5f, 0);
                                    Wait(0.8);
                                } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                    counter++;
                                    currentState = states.Silver;
                                    telemetry.addData("Counter ", counter);
                                    telemetry.addData("State: ", "Silver");
                                    telemetry.update();
                                    Wait(0.8);
                                }
                            }
                        }
                        break;
                    case Gold:
                        updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions == null) {
                            currentState = states.Space;
                            telemetry.addData("State: ", "Space");
                            telemetry.update();
                        }
                        break;
                    case Silver:
                        if (updatedRecognitions == null) {
                            currentState = states.Space;
                            telemetry.addData("State: ", "Space");
                            telemetry.update();
                        }
                        break;
                    default:
                        telemetry.addData("STATE INCORRECT", " AHHH ");
                        telemetry.update();
                        break;
                }
                if (counter == 3) {
                    sR();
                    telemetry.addData("WE Stopped Robot Cause Counter =  ", counter);
                    telemetry.update();
                    tfod.deactivate();
                    return;
                }
            }
        }
    }

    void tensorFlowJeffrey() {
        tfod.activate();
        List<Recognition> updatedRecognitions;
        int counter = 0;
        while (true) {
            adjustHeading(0, 0.5f, 0);
            if (tfod != null) {
                updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    for (Recognition recognition : updatedRecognitions) {
                        if ((recognition.getLabel().equals(LABEL_SILVER_MINERAL)) || (recognition.getLabel().equals((LABEL_GOLD_MINERAL)))) {
                            counter++;
                            if (!(counter == 3)) {
                                Wait(0.8);
                            }
                        }
                    }
                }
                if (counter == 3) {
                    return;
                }
            }
        }
    }

    void tensorFlowCount() {
        tfod.activate();
        runtime.reset();
        telemetry.setAutoClear(true);
        translateForever(1, 0, 0.7);
        int counter = 0;
        boolean space = true;
        int gold = 0;

        while (true) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    // telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() >= 1) {
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) || recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                if (space) {
                                    counter++;
                                    space = false;
                                    telemetry.addData("Counter: ", counter);
                                    telemetry.addData("Space: ", space);
                                }
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    gold++;
                                    if (gold == 1) {
                                        telemetry.addData("Gold Mineral ", "Visible");
                                        sR();
                                        followHeading(0, 0.1, 0, 1);
                                        followHeading(0, 0.1, 0, -1);

                                        //Pick up mineral
                                        translateForever(1, 0, 0.7);
                                    }
                                }
                                if (counter == 3) {
                                    sR();
                                    tfod.deactivate();
                                    return;
                                }
                            } else {
                                space = true;
                                telemetry.addData("Gold Mineral/Silver Mineral ", "Not Visible");
                            }
                            telemetry.update();
                        }
                    }
                }
            }
        }
    }

    enum pivotStates {None, Left, Right, Init}

    enum states {Space, Gold, Gold_Again, Silver, Silver_Again}
}