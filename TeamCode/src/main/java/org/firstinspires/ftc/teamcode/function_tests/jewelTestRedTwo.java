package org.firstinspires.ftc.teamcode.function_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Jewel;


@Autonomous(name = "JewelTestRed 2",group = "a")
public class jewelTestRedTwo extends AutonomousOpMode {

    @Override
    public void runOpMode() {

        initit();

        waitForStart();

        hitRedJewel2();

    }
}
