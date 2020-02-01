package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: MoveOnlyBlue2Near", group="Preciousss")

public class MoveOnlyBlue2Near extends superAutoNew {

    public void runOpMode() {
        setUp();

        double sideSpeed=14;
        double forwardSpeed=24.5;

        //configVuforiaSkystone();

        playSound(ss_light_saber);

        Park(-1d);

        sR();


    }

}
