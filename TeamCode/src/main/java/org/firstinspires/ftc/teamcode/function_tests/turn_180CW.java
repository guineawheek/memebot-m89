package org.firstinspires.ftc.teamcode.function_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Gyro;

@Autonomous(name = "180CW",group = "a")
public class turn_180CW extends LinearOpMode {
    DriveTrain motors = null;
    Gyro gyro = null;
    @Override
    public void runOpMode() {
        motors = new DriveTrain(hardwareMap, telemetry);
        gyro = new Gyro(hardwareMap, telemetry);

        waitForStart();

        motors.Turn(-180,gyro);

    }
}
