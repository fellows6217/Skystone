package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: AutoMoveOnlyBlue2", group="Preciousss")

    public class AutoMoveOnlyBlue2 extends superAutoNew {

    public void runOpMode() {
        setUp();
        //configVuforiaSkystone();

        playSound(ss_light_saber);

        Park(-1d);

        sR();


    }

}
