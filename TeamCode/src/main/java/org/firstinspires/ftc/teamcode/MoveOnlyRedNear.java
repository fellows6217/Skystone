package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: MoveOnlyRedNear", group="Preciousss")

public class MoveOnlyRedNear extends superAutoNew {

        public void runOpMode() {
            setUp();
            //configVuforiaSkystone();

            double sideSpeed=14;
            double forwardSpeed=24.5;

            playSound(ss_power_up);

            translate(-1,0,2/sideSpeed,0.75);

            moveFoundation(-1d);

            //Translate
            translate(1,0,58/sideSpeed,0.75);

            sR();



        }

}
