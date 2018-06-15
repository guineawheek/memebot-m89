package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Gyro;
import org.firstinspires.ftc.teamcode.subsystems.Jewel;
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.Turn;

/**
 * Created by xiax on 4/23/2018.
 */
@Disabled
@Autonomous(name = "mv24dir0", group = "AAAA")
public class TestAuton extends LinearOpMode {
    //red
    @Override
    public void runOpMode() {

        Turn motors = null;
        Gyro gyro = null;
        Jewel jewel;
        Servos servos;

        motors = new Turn(hardwareMap, telemetry);
        gyro = new Gyro(hardwareMap, telemetry);
        jewel = new Jewel(hardwareMap, telemetry);
        servos = new Servos(hardwareMap);

        telemetry.addData("Status", "Turn Initialized");
        telemetry.update();

        waitForStart();
        jewel.hitRedJewel();
        //scan vumark
        motors.MoveTo(20, 180, .5);

        motors.MoveTo(1/*vu*/, 90, 0.5);
        //forward based on range
        servos.setAutoAlign(true);
        //sideways based on touch sensor
        servos.setFlipperGrab(true);
        servos.setFlipperUp(true);
        servos.setFlipperGrab(false);
        sleep(50);

        motors.MoveTo(5, 0, .5);

        motors.MoveTo(3, 180, .5);



    }
}
