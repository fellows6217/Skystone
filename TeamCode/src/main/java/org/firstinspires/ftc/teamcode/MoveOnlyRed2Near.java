package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: MoveOnlyRed2Near", group="Preciousss")

    public class MoveOnlyRed2Near extends superAutoNew {

    public void runOpMode() {
        setUp();
        //configVuforiaSkystone();

        playSound(ss_light_saber);

        Park(1d);

        sR();


    }

}
