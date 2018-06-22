package org.firstinspires.ftc.teamcode.Autonomouses;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;

/**
 * Created by xiax on 4/23/2018.
 */
@Disabled
@Autonomous(name = "Close Red 2", group = "Autonomous")
public class CloseRedTwo extends AutonomousOpMode {

    DcMotor mtrIntakeLeft;
    DcMotor mtrIntakeRight;

    @Override
    public void runOpMode() {

        mtrIntakeLeft = hardwareMap.dcMotor.get("mtrIntakeLeft");
        mtrIntakeRight = hardwareMap.dcMotor.get("mtrIntakeRight");

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
            MoveToByEncoder(32.25, 180, .3);
        } else if (column == 3) {
            MoveToByEncoder(23, 180, .3);
        } else {
            MoveToByEncoder(29.75, 180, .3);
        }

        sleep(100);

        Turn(-90, gyro);

        alignCryptoSequence();

        depositGlyphNormal();

        mtrIntakeLeft.setPower(1);
        mtrIntakeRight.setPower(1);

        sleep(100);

        mtrIntakeLeft.setPower(-1);
        mtrIntakeRight.setPower(-1);

        MoveToByEncoder(30, 0, .3);

        MoveToByEncoder(22, 180, .3);

        if (column == 1) {
            MoveToByEncoder(10, 270, .3);
        } else {
            MoveToByEncoder(10 ,90,.3);
        }
        MoveToByTime(500,180,.3);

        alignCryptoSequence();

        depositGlyphNormal();

    }
}
