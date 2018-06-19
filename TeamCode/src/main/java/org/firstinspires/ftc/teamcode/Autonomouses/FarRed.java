package org.firstinspires.ftc.teamcode.Autonomouses;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Gyro;
import org.firstinspires.ftc.teamcode.subsystems.Jewel;
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.VuMarkRecognition;

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

        if (column == 1) //left
            MoveToByEncoder(18.5, 90, .3);//19 in
        else if (column == 3) //right
            //MoveToByEncoder(10, 90, .3);//5 in
            opModeIsActive();
        else //center
            MoveToByEncoder(10, 90, .3);//12 in

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
