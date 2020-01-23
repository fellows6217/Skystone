package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: AutoMoveOnlyRed", group="Preciousss")

public class AutoMoveOnlyRed extends superAutoNew {

        public void runOpMode() {
            setUp();
            //configVuforiaSkystone();

            double sideSpeed=14;

            playSound(ss_power_up);

            //Translate
            translate(1,0,1.5/sideSpeed, 0.75);

            moveFoundation(-1d);

            sR();



        }

}
