package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Gyro;
import org.firstinspires.ftc.teamcode.subsystems.Jewel;
import org.firstinspires.ftc.teamcode.subsystems.Turn;

/**
     * Created by xiax on 4/23/2018.
     */
    @Disabled
    @Autonomous(name = "mv24dir0", group = "AAAA")
    public class TestAuton extends LinearOpMode {
//red
        @Override public void runOpMode() {

            Turn motors = null;
            Gyro gyro = null;
            Jewel jewel;

            motors = new Turn(hardwareMap, telemetry);
            gyro = new Gyro(hardwareMap, telemetry);
            jewel = new Jewel(hardwareMap, telemetry);

            telemetry.addData("Status", "Turn Initialized");
            telemetry.update();

            waitForStart();

            //jewel
                jewel.hitRedJewel();
            //forward
                motors.MoveTo(999,0,.5);//if, else if, else depending on column
            //turn
                motors.Turn(90,gyro);
            //bring out building detector
                //svoAuto.setposition(OUT)
            //align
                //motors.moveBasedOnTime(Dirction: 90 , Time: 2000)
            //deposit
                //deposit sequence
            //collect
                motors.MoveTo(999,0,.5);
                //collector motors on

            //bring out building detector
                //svoAuto.setposition(OUT);
            //align
                //motors.moveBasedOnTime(Dirction: 90 , Time: 2000)
            //REPEAT (not using loop function)

        }
    }
