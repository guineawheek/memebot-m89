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
@Disabled
@Autonomous(name = "testauton", group = "Autonomous")
public class TestAuton extends AutonomousOpMode {
    // far red
    DriveTrain motors = null;
    Gyro gyro = null;
    Jewel jewel;
    Servos servos;
    VuMarkRecognition vuMark;
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
        waitForStart();

        int column = vuMark.getVuMark();

        servos.setRelicPivotGrab(false);

        jewel.hitRedJewel(); //all steps of jewels

        //all distances unknown
        MoveToByEncoder(24, 180, .5); //off the stone

        if (column == 1) //left
            MoveToByEncoder(22, 270, .5);//19 in
        else if (column == 3) //right
            MoveToByEncoder(8, 270, .5);//5 in
        else //center
            MoveToByEncoder(15, 270, .5);//12 in

        MoveToByRange(16/*cm*/,180,.5); //forwards based on range

        servos.setAutoAlign(true); //raise alignment

        MoveToBySwitch(270,.5);//strafe until alignment hit

        sleep (50);

        servos.setAutoAlign(false); //lower alignment

        servos.setFlipperUp(true);//deposit
        sleep (100);
        servos.setFlipperGrab(false);
        sleep(50);

        MoveToByTime(200, 0, .5); //little nudge for block
        MoveToByTime(300, 180, .5);

        servos.setFlipperUp(false);

        MoveToByEncoder(5,0,.5); //back out - maybe for more? - park in box

    }
}
