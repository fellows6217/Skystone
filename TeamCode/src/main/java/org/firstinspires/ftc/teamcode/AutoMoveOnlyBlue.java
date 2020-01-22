package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: AutoMoveOnlyBlue", group="Preciousss")

public class AutoMoveOnlyBlue extends superAutoNew {

        public void runOpMode() {
            setUp();
            //configVuforiaSkystone();

            playSound(ss_light_saber);

            moveFoundation(1d);

            sR();



        }

}
