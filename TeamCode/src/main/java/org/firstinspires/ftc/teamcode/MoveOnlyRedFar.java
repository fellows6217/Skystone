package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: MoveOnlyRedFar", group="Preciousss")

public class MoveOnlyRedFar extends superAutoNew {

    public void runOpMode() {
        setUp();
        //configVuforiaSkystone();

        double sideSpeed=14;
        double forwardSpeed=24.5;

        playSound(ss_power_up);

        //translate(-1,0,2/sideSpeed,0.75);

        moveFoundation(-1d);

        //Translate
        translate(1,0,42/sideSpeed,0.75);

        translate(0,1,20/forwardSpeed,0.75);

        translate(1,0,18/sideSpeed,0.75);

        sR();

        //translate a total of 48 inches to make it back to the skybridge



    }

}
