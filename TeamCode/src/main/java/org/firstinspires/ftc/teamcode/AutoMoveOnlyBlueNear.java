package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: AutoMoveOnlyBlueNear", group="Preciousss")

public class AutoMoveOnlyBlueNear extends superAutoNew {

        public void runOpMode() {

            double sideSpeed=14;
            double forwardSpeed=24.5;

            setUp();
            //configVuforiaSkystone();

            playSound(ss_light_saber);

            moveFoundation(1d);

            //Translate
            translate(-1, 0, 48/sideSpeed, 0.75);

            sR();



        }

}
