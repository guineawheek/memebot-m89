package org.firstinspires.ftc.teamcode.Autonomouses;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;

/**
 * Created by xiax on 4/23/2018.
 */

@Autonomous(name = "Close Blue", group = "Autonomous")
public class CloseBlue extends AutonomousOpMode {

    @Override
    public void runOpMode() {

        initit();

        int column = 0;
        while (!isStarted() && !isStopRequested()) {
            column = vuMark.getVuMark();
            telemetry.addData("Vumark: ", column);
            telemetry.update();
        }

        servos.setRelicPivotGrab(false);

        hitRedJewel(); //all steps of jewels

        //all distances unknown

        mtrRelic.setPower(.2);

        MoveToByEncoder(20, 0, .3); //off the stone

        sleep(200);// allow time for relic extension to retract

        Turn(90,gyro);

        if (column == 1) //left
            MoveToByEncoder(10, 270, .3);//19 in
        else if (column == 3) //right
            MoveToByEncoder(32, 270, .3);//5 in
        else //center
            MoveToByEncoder(18.5, 270, .3);//12 in

        servos.setAutoAlign(true);//raise alignment

    //   MoveToByRange(16/*cm*/,180,1); //forwards based on range

        MoveToBySwitch(90,.3);//strafe until alignment hit

        sleep (50);

        servos.setAutoAlign(false); //lower alignment

        servos.setFlipperUp(true);//deposit
        sleep (1000);

        servos.setFlipperGrab(false);
        sleep(50);
       // if (true) return;
        MoveToByTime(500, 0, .3); //little nudge for block
        MoveToByTime(1000, 180, .5);

        servos.setFlipperUp(false);

        MoveToByEncoder(5,0,.5); //back out - maybe for more? - park in box

    }
}
