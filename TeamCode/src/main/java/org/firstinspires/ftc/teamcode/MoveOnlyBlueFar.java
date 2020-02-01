package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Preciousss: MoveOnlyBlueFar", group="Preciousss")

public class MoveOnlyBlueFar extends superAutoNew {

    public void runOpMode() {
        setUp();
        //configVuforiaSkystone();

        double sideSpeed=14;
        double forwardSpeed=24.5;

        playSound(ss_power_up);

        moveFoundation(1d);

        //Translate
        translate(-1,0,16/sideSpeed,0.75);

        translate(0,1,26/forwardSpeed,0.75);

        translate(-1,0,40/sideSpeed,0.75);

        sR();

        //translate a total of 56 inches to make it back to the skybridge



    }

}
