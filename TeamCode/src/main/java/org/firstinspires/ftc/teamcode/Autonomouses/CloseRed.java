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

        if (column == 1) {
            MoveToByEncoder(35.5, 180, .3);
        } else if (column == 3) {
            MoveToByEncoder(21.5, 180, .3);
        } else {
            MoveToByEncoder(30, 180, .3);
        }

        sleep(100);

        Turn(-90, gyro);

        alignCryptoSequence();

/*        if (column == 1) //left
            MoveToByEncoder(14.5, 90, .3);//19 in
        else if (column == 3) //right
            MoveToByEncoder(7, 300, .3);
            //opModeIsActive();
        else //center
            MoveToByEncoder(5, 90, .3);//12 in
*/


        depositGlyphNormal();
    }
}
