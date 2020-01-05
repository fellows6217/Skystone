package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: AutoMoveOnlyRed", group="Preciousss")

public class AutoMoveOnlyRed extends superAutoNew {

        public void runOpMode() {
            setUp();
            //configVuforiaSkystone();

            playSound(ss_power_up);

            moveFoundation(1d);

            sR();



        }

}
