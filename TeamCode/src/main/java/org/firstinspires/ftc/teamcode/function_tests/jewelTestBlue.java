package org.firstinspires.ftc.teamcode.function_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Jewel;

@Disabled
@Autonomous(name = "JewelTestBlue",group = "a")
public class jewelTestBlue extends AutonomousOpMode {

    @Override
    public void runOpMode() {

        initit();

        waitForStart();

        hitBlueJewel();

    }
}
