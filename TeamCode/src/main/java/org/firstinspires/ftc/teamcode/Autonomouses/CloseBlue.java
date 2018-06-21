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


        if (column == 1) {
            MoveToByEncoder(32.25, 0, .3);
        } else if (column == 3) {
            MoveToByEncoder(48, 0, .3);
        } else {
            MoveToByEncoder(40.75, 0, .3);
        }

        sleep(200);// allow time for relic extension to retract

        Turn(-90, gyro);

        alignCryptoSequence();

        depositGlyphNormal();

    }
}
