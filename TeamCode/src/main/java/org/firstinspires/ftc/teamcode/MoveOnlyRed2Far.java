package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: MoveOnlyRed2Far", group="Preciousss")

public class MoveOnlyRed2Far extends superAutoNew {

    public void runOpMode() {
        setUp();

        double sideSpeed=14;
        double forwardSpeed=24.5;

        //configVuforiaSkystone();

        playSound(ss_light_saber);

        //Forward
        translate(0,-1,20/forwardSpeed,0.75);

        Park(-1d);

        sR();


    }

}
