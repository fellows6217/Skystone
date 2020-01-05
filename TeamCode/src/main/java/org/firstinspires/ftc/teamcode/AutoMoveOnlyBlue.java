package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: AutoMoveOnlyBlue", group="Preciousss")

public class AutoMoveOnlyBlue extends superAutoNew {

        public void runOpMode() {
            setUp();
            //configVuforiaSkystone();

            playSound(ss_bb8_up);

            moveFoundation(-1d);

            sR();



        }

}
