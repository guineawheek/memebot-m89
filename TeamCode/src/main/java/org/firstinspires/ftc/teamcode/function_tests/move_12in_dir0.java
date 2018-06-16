package org.firstinspires.ftc.teamcode.function_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Gyro;
import org.firstinspires.ftc.teamcode.subsystems.Jewel;
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.VuMarkRecognition;

@Autonomous(name = "12in dir0",group = "a")
public class move_12in_dir0 extends LinearOpMode {
    DriveTrain motors = null;
    Gyro gyro = null;
    @Override
    public void runOpMode() {
        motors = new DriveTrain(hardwareMap, telemetry);
        gyro = new Gyro(hardwareMap, telemetry);

        waitForStart();

        motors.MoveTo(12,0,.5);

    }
}
