/* Team 6217
    The Fellowship
    Test Robot Code */

package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.ftccommon.SoundPlayer;
import android.content.Context;
import android.media.AudioManager;
import android.media.ToneGenerator;

import static java.lang.Math.PI;
import static java.lang.Math.abs;




@TeleOp(name="Preciousss: TeleOpTestBot", group="Preciousss")

public class TeleOpTestBot extends OpMode
{
    //*******************************************************************************************
    //Drive motors
    /*FR = Front Right Wheel, FL = Front Left Wheel, BR = Back Right Wheel, BL = Back Left Wheel, Con1= Conveyor 1, Con2= Conveyor 2*/
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBR;
    DcMotor motorBL;

    DcMotor reacher;
    DcMotor slideLifter;

    Servo wrist;
    Servo claw;

    DigitalChannel digitalTouch;  // Hardware Device Object


    /* IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;*/

    //static final double     COUNTS_PER_MOTOR_REV    = 420 ; // PPR for NeveRest 400
    //static final double     limitRocker = COUNTS_PER_MOTOR_REV / 4 ; // 90 degrees
    private ElapsedTime     runtime = new ElapsedTime();

    private boolean RunLauncher = false;
    private int DelayCounter = 0;
    private int DelayTimer = 400 ;
    //public TeleOpTestBot() {}

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

    @Override
    public void init()
    {
        //*******************************************************************************************
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

        reacher = hardwareMap.dcMotor.get("reacher");
        reacher.setDirection(DcMotor.Direction.FORWARD);
        slideLifter = hardwareMap.dcMotor.get("slideLifter");
        slideLifter.setDirection(DcMotor.Direction.FORWARD);

        wrist = hardwareMap.servo.get("wrist");
        claw = hardwareMap.servo.get("claw");

        // get a reference to our digitalTouch object.
      //  digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
      //  digitalTouch.setMode(DigitalChannel.Mode.INPUT);


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // Start wrist in stowed position
        //servoWrist.setPosition(.85);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */


    @Override
    public void loop() {

        /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Read values from Game Controller
            ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

        // Stick values range from -1 to 1, where -1 is full up, and 1 is full down
        // Trigger values range from 0 to 1

        /* posx = Position X on Left Joystick, posy = Position Y on Left Joystick, posxR = Position X on Right Joystick,
        posyR = Position Y on Right Joystick, LT = Left Trigger, RT = Right Trigger, a = Button a, y = Button y, x = Button x,
         b = Button b, UP = on Dpad Loads Glyph, DOWN = on Dpad Shoots Glyph, rightPad = Right on Dpad, leftPad = Left on Dpad */
        float FLBRPower = 0.f;
        float FRBLPower = 0.f;
        float posx = gamepad1.left_stick_x;
        float posy = gamepad1.left_stick_y;
        float LT = gamepad1.left_trigger;
        float RT = gamepad1.right_trigger;
        boolean a = gamepad1.a; // open/close grabber
        boolean b = gamepad1.b; // wrist 80
        boolean y = gamepad1.y; // wrist 90
        boolean x = gamepad1.x; // wrist 0
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
         /* ~~~~~~~~~~ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Limit x and y values and adjust to power curve
            ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

        posx = Range.clip(posx, -1, 1);
        posy = Range.clip(posy, -1, 1);

        posx = (float) powerCurve(posx);
        posy = (float) powerCurve(posy);

        LT = (float) powerCurve(LT);
        RT = (float) powerCurve(RT);


        // pivot left
        if (LT != 0) {

            motorFL.setPower(-1);
            motorBL.setPower(-1);
            motorFR.setPower(1);
            motorBR.setPower(1);
        }


        //  pivot right
        else if (RT != 0) {

            motorFL.setPower(1);
            motorBL.setPower(1);
            motorFR.setPower(-1);
            motorBR.setPower(-1);
        }


        //  Driving
        if ((posy != 0) || (posx != 0)) {

            FRBLPower = -posy - posx;
            FLBRPower = -posy + posx;
            motorFR.setPower(FRBLPower);
            motorFL.setPower(FLBRPower);
            motorBR.setPower(FLBRPower);
            motorBL.setPower(FRBLPower);

        } else {

            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);
        }

        // Grabber controls

        if (dpadUp) { //slide up
            slideLifter.setPower(1);
        } else if (dpadDown) { //slide down
            slideLifter.setPower(-1);
        } else if (dpadLeft) { // reacher in
            reacher.setPower(-1);
        } else if (dpadRight) { // reacher out
            reacher.setPower(1);
        } else {
            slideLifter.setPower(0);
            reacher.setPower(0);
        }

        /*if (a) {
            claw.setPosition(180.); // closed
        } else {
            claw.setPosition(0); // open
        }*/

        if (a) {
            if (claw.getPosition() > 0.1) {
                claw.setPosition(0);
                //playSound(7);
            } else {
                claw.setPosition(1);
                //playSound(10);
            }
            Wait(0.25d);
        }

        /* if (b) {
            wrist.setPosition(80.);
        } else if (y) {
            wrist.setPosition(90);
        } else if (x) {
            wrist.setPosition(0);
        }
        */

        /* Incremental wrist turning */

        if (y) {
            wrist.setPosition(0.5);
        } else if (b) {
            wrist.setPosition(wrist.getPosition() - 0.001);
        } else if (x) {
            wrist.setPosition(wrist.getPosition() + 0.001);
        }

        /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Write telemetry back to driver station
            NOTE: Output is sorted by the first field, hence the numbering
            ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */


        telemetry.addData("1", "Power, FRBL/FLBR: " +
                String.format("%.2f", FRBLPower) + " " +
                String.format("%.2f", FLBRPower));

        telemetry.addData("2", "Raw Joystick X, Y: " +
                String.format("%.2f", gamepad1.left_stick_x) + " " +
                String.format("%.2f", gamepad1.left_stick_y));

        telemetry.addData("3", "D-Pad U, R, D, L: " +
                String.format("%b", gamepad1.dpad_up) + " " +
                String.format("%b", gamepad1.dpad_right) + " " +
                String.format("%b", gamepad1.dpad_down) + " " +
                String.format("%b", gamepad1.dpad_left));

        telemetry.addData("0", "Trigger: " +
                String.format("%.2f", gamepad1.left_trigger));


        telemetry.addData("0", "Trigger: " +
                String.format("%.2f", gamepad1.right_trigger));
        telemetry.addData("4", "Right Bumper Left Bumper" +
                String.format("%b", gamepad1.right_bumper)+ " " +
                String.format("%b", gamepad1.left_bumper));

        telemetry.addData("5", "Actuator A, B,: " +
                String.format("%b", gamepad1.a) + " " +
                String.format("%b", gamepad1.b));

        /*if (digitalTouch.getState() == true) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
        }*/
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {
    }

    double powerCurve ( float dVal ) {
        /*
         * This method adjusts the joystick input to apply a power curve that
         * is less than linear.  This is to make it easier to drive
         * the robot more precisely at slower speeds.
         */
        // LB = left bumper, RB = right bumper.
        boolean LB = gamepad1.left_bumper;
        boolean RB = gamepad1.right_bumper;


        double dead = .2;

        // Regular
        double max = .6;
        if (LB == true && RB == false){
            // Turbo with left bumper
            max = 1;
        } else if (LB == false && RB == true){
            // Turtle with right bumper
            max = .3;
        }

        int direction = 1;
        if (dVal < 0) {
            direction = -1;
        }

        double dScale = 0.0;
        if (abs(dVal) < dead) {
            dScale = 0.0;
        } else {
            dScale = ((dVal * dVal) * max) * direction;
        }

        return (dScale);
    }

    void Wait(double WaitTime) {
        runtime.reset();
        while ((runtime.seconds() < WaitTime)) {
        }
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

}

