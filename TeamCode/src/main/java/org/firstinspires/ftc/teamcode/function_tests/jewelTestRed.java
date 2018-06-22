package org.firstinspires.ftc.teamcode.function_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Gyro;
import org.firstinspires.ftc.teamcode.subsystems.Jewel;

@Disabled
@Autonomous(name = "JewelTestRed",group = "a")
public class jewelTestRed extends AutonomousOpMode {

    @Override
    public void runOpMode() {
        jewel = new Jewel(hardwareMap,telemetry);
        initit();

        waitForStart();

        hitRedJewel();

    }
}
