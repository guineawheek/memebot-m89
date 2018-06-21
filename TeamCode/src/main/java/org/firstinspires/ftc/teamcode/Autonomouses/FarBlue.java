package org.firstinspires.ftc.teamcode.Autonomouses;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;

/**
 * Created by xiax on 4/23/2018.
 */

@Autonomous(name = "Far Blue", group = "Autonomous")
public class FarBlue extends AutonomousOpMode {

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

        MoveToByEncoder(29, 0, .3); //off the stone

        sleep(200);// allow time for relic extension to retract

        Turn(180,gyro);

        if (column == 1) //left
            MoveToByEncoder(18, 270, .3);//19 in
        else if (column == 3) //right
            MoveToByEncoder(42, 270, .3);//5 in
        else //center
            MoveToByEncoder(26.75, 270, .3);//25?

        alignCryptoSequence();

        depositGlyphNormal();
    }
}
