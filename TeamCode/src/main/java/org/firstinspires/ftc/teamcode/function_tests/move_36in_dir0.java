package org.firstinspires.ftc.teamcode.function_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Gyro;

@Autonomous(name = "36in dir0",group = "a")
public class move_36in_dir0 extends LinearOpMode {
    DriveTrain motors = null;
    Gyro gyro = null;
    @Override
    public void runOpMode() {
        motors = new DriveTrain(hardwareMap, telemetry);
        gyro = new Gyro(hardwareMap, telemetry);

        waitForStart();

        motors.MoveToByEncoder(36,0,.5);

    }
}
