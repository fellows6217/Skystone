package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: AutoMoveOnlyRed2", group="Preciousss")

    public class AutoMoveOnlyRed2 extends superAutoNew {

    public void runOpMode() {
        setUp();
        //configVuforiaSkystone();

        playSound(ss_light_saber);

        Park(1d);

        sR();


    }

}
