package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: MoveOnlyBlue2Far", group="Preciousss")

public class MoveOnlyBlue2Far extends superAutoNew {

    public void runOpMode() {
        setUp();

        double sideSpeed=14;
        double forwardSpeed=24.5;

        //configVuforiaSkystone();

        playSound(ss_light_saber);

        Park(-1d);

        //Translate
        translate(-1,0,16/sideSpeed,0.75);

        translate(0,1,26/forwardSpeed,0.75);

        translate(-1,0,40/sideSpeed,0.75);


        sR();


    }

}
