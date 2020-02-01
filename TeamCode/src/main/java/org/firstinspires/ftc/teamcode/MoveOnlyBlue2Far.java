package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: MoveOnlyBlue2Far", group="Preciousss")

    public class MoveOnlyBlue2Far extends superAutoNew {

    public void runOpMode() {
        setUp();

        //configVuforiaSkystone();

        double sideSpeed=14;
        double forwardSpeed=24.5;

        playSound(ss_light_saber);

        //Forward
        translate(0,-1,20/forwardSpeed,0.75);

        Park(-1d);

        sR();


    }

}
