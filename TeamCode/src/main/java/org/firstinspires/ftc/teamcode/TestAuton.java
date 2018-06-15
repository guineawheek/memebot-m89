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
@Autonomous(name = "mv24dir0", group = "AAAA")
public class TestAuton extends LinearOpMode {
    // far red
    DriveTrain motors = null;
    Gyro gyro = null;
    Jewel jewel;
    Servos servos;
    VuMarkRecognition vuMark;
    @Override
    public void runOpMode() {
        motors = new DriveTrain(hardwareMap, telemetry);
        gyro = new Gyro(hardwareMap, telemetry);
        jewel = new Jewel(hardwareMap, telemetry);
        servos = new Servos(hardwareMap);

        vuMark = new VuMarkRecognition(this.hardwareMap, this.telemetry);

        servos.init(); //auto version - need to do - make sure to grip the block

        telemetry.addData("Status", "DriveTrain Initialized");
        telemetry.update();

        waitForStart();

        int column = vuMark.getVuMark();

        jewel.hitRedJewel(); //all steps of jewels

        //all distances unknown
        motors.MoveTo(20, 180, .5); //off the stone

        if (column == 1) //left
            motors.MoveTo(1, 270, .5);
        else if (column == 3) //right
            motors.MoveTo(3, 270, .5);
        else //center
            motors.MoveTo(2, 270, .5);
        motors.MoveToByRange(16/*cm*/,180,.5); //forwards based on range

        servos.setAutoAlign(true); //raise alignment

        motors.MoveToBySwitch(270,.5);//strafe until alignment hit
        sleep (50);
        servos.setAutoAlign(false); //lower alignment

        servos.setFlipperUp(true);//deposit
        sleep (100);
        servos.setFlipperGrab(false);
        sleep(50);

        motors.MoveToByTime(100, 0, .5); //little nudge for block
        motors.MoveToByTime(200, 180, .5);

        servos.setFlipperUp(false);

        motors.MoveTo(5,0,.5); //back out - maybe for more? - park in box

    }
}
