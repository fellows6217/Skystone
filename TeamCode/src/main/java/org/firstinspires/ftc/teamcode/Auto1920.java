package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: Auto1920", group="Preciousss")

public class Auto1920 extends superAutoNew {

        public void runOpMode() {
            setUp();
            //configVuforiaSkystone();

            playSound(14);


            //Raise arm a little
            slideLifter.setPower(1);
            Wait(0.5d);
            slideLifter.setPower(0);

            //Translate
            translate(-1,0,3, 0.75);

            //Reach out
            reacher.setPower(1);
            Wait(2.75d);
            reacher.setPower(0);

            //Set claw down
            slideLifter.setPower(1);
            Wait(0.5);
            slideLifter.setPower(0);

            //Translate
            translate(1, 0, 3, 0.75);

            //Back up to park under bridge
            translate(0,1,2,0.75);


               sR();



        }

}
