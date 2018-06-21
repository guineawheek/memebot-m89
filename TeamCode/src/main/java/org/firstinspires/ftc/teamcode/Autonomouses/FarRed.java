package org.firstinspires.ftc.teamcode.Autonomouses;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;

/**
 * Created by xiax on 4/23/2018.
 */

@Autonomous(name = "Far Red", group = "Autonomous")
public class FarRed extends AutonomousOpMode {
    // far red
    //  VuMarkRecognition vuMark;
    @Override
    public void runOpMode() {
       /* motors = new DriveTrain(hardwareMap, telemetry);
        gyro = new Gyro(hardwareMap, telemetry);
        jewel = new Jewel(hardwareMap, telemetry);
        servos = new Servos(hardwareMap);

        vuMark = new VuMarkRecognition(this.hardwareMap, this.telemetry);

        servos.autoInit();

        jewel.jewelStow();

        telemetry.addData("Status", "DriveTrain Initialized");
        telemetry.update();
*/
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
        MoveToByEncoder(29.5, 180, .3); //off the stone

        if (column == 1) { //left
            MoveToByEncoder(18.5, 90, .3);//19 in
            alignCryptoSequence();
        } else if (column == 3) //right
            opModeIsActive();
        else //center
            MoveToByEncoder(10, 90, .3);//12 in

        depositGlyphNormal();
    }
}
