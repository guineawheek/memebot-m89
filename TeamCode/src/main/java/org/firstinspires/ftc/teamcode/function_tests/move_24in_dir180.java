package org.firstinspires.ftc.teamcode.function_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Gyro;

@Autonomous(name = "24in dir180",group = "a")
public class move_24in_dir180 extends LinearOpMode {
    DriveTrain motors = null;
    Gyro gyro = null;
    @Override
    public void runOpMode() {
        motors = new DriveTrain(hardwareMap, telemetry);
        gyro = new Gyro(hardwareMap, telemetry);

        waitForStart();

        motors.MoveTo(24,180,.5);

    }
}
