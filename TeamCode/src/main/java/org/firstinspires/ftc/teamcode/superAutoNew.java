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
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    DcMotor motorBL;
    //ModernRoboticsI2cRangeSensor rangeSensor;


    //*******************************************************************************************
    //Devices Specific to Challenge


    DcMotor reacher;
    DcMotor slideLifter;

    Servo wrist;
    Servo claw;


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



       // initTfod();



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
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFL.setDirection(DcMotor.Direction.FORWARD);

        Boolean fullRobot = false;


        //+++++++++++++++++++++++++++++++++++++++++++++++++
        //Devices Specific to Challenge


        reacher = hardwareMap.dcMotor.get("reacher");
        reacher.setDirection(DcMotor.Direction.FORWARD);
        slideLifter = hardwareMap.dcMotor.get("slideLifter");
        slideLifter.setDirection(DcMotor.Direction.FORWARD);

        wrist = hardwareMap.servo.get("wrist");
        claw = hardwareMap.servo.get("claw");

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

        double FRBLPower = -basePosy - basePosx;
        double FLBRPower = -basePosy + basePosx;
        motorFR.setPower(FRBLPower + addPower);
        motorFL.setPower(FLBRPower - addPower);
        motorBR.setPower(FLBRPower + addPower);
        motorBL.setPower(FRBLPower - addPower);
    }

    void translate(double posx, double posy, double seconds, double power) {
        double FRBLPower = (-posy - posx) * power;
        double FLBRPower = (-posy + posx) * power;
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
        double FRBLPower = (-posy - posx) * power;
        double FLBRPower = (-posy + posx) * power;
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

    void moveFoundation(double direction) {

        double sideSpeed=14;
        double forwardSpeed=24.5;

        //Raise arm a little
        slideLifter.setPower(1);
        Wait(0.5d);
        slideLifter.setPower(0);

        //Translate
        translate(direction,0,15/sideSpeed, 0.75);

        //Forward
        translate(0, -1,30/forwardSpeed,0.75);

        //Reach out
        reacher.setPower(1);
        Wait(3.5);
        reacher.setPower(0);

        //Rotate wrist
        wrist.setPosition(0.8);
        Wait(1.5d);

        //Set claw down
        slideLifter.setPower(-1);
        Wait(0.5d);
        slideLifter.setPower(0);
        Wait(0.5d);

        //Back up to drag foundation
        translate(0,1,3,0.5);

        //Lift claw
        slideLifter.setPower(1);
        Wait(0.5d);
        slideLifter.setPower(0);
        Wait(0.5d);

        //Translate
        translate(-direction, 0, 48/sideSpeed, 0.75);

    }

    enum pivotStates {None, Left, Right, Init}

    enum states {Space, Gold, Gold_Again, Silver, Silver_Again}
}