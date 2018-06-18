package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Gyro;
import org.firstinspires.ftc.teamcode.subsystems.Jewel;
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.VuMarkRecognition;

/**
 * Created by xiax on 4/23/2018.
 */

@Autonomous(name = "testauton", group = "Autonomous")
public class TestAuton extends AutonomousOpMode {
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
        MoveToByEncoder(32, 180, .4); //off the stone
        if (true) return;
        if (column == 1) //left
            MoveToByEncoder(24, 90, .4);//19 in
        else if (column == 3) //right
            MoveToByEncoder(10, 90, .4);//5 in
        else //center
            MoveToByEncoder(17, 90, .5);//12 in

        servos.setAutoAlign(true);//raise alignment

    //   MoveToByRange(16/*cm*/,180,1); //forwards based on range

        MoveToBySwitch(90,.3);//strafe until alignment hit

        sleep (50);

        servos.setAutoAlign(false); //lower alignment

        servos.setFlipperUp(true);//deposit
        sleep (100);
        servos.setFlipperGrab(false);
        sleep(50);

        MoveToByTime(500, 0, .3); //little nudge for block
        MoveToByTime(1000, 180, .5);

        servos.setFlipperUp(false);

        MoveToByEncoder(5,0,.5); //back out - maybe for more? - park in box

    }
}
