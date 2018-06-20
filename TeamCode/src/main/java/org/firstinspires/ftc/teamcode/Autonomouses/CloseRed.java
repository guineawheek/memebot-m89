package org.firstinspires.ftc.teamcode.Autonomouses;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;

/**
 * Created by xiax on 4/23/2018.
 */

@Autonomous(name = "Close Red", group = "Autonomous")
public class CloseRed extends AutonomousOpMode {
    // far red
    //  VuMarkRecognition vuMark;
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

     hitBlueJewel(); //all steps of jewels

        //all distances unknown
        MoveToByEncoder(20, 180, .3); //off the stone

        Turn(-90, gyro);

        MoveToByEncoder(7,180,.3);

        if (column == 1) //left
            MoveToByEncoder(14.5, 90, .3);//19 in
        else if (column == 3) //right
            MoveToByEncoder(3, 330,.3);
            //opModeIsActive();
        else //center
            MoveToByEncoder(4, 90, .5);//12 in

        servos.setAutoAlign(true);//raise alignment

        //   MoveToByRange(16/*cm*/,180,1); //forwards based on range

        MoveToBySwitch(90, .3);//strafe until alignment hit

        sleep(50);

        servos.setAutoAlign(false); //lower alignment

        servos.setFlipperUp(true);//deposit
        sleep(1000);

        servos.setFlipperGrab(false);
        sleep(50);
        // if (true) return;
        MoveToByTime(700, 0, .3); //little nudge for block

        MoveToByTime(1100, 180, .5);



        MoveToByEncoder(5, 0, .5); //back out - maybe for more? - park in box

        servos.setFlipperUp(false);

    }
}
