package org.firstinspires.ftc.teamcode.function_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.subsystems.AndroidGyro;
import org.firstinspires.ftc.teamcode.subsystems.Jewel;

@Disabled
@Autonomous(name = "GyroTest",group = "a")
public class gyroTest extends AutonomousOpMode {

    @Override
    public void runOpMode() {
        AndroidGyro gyro = new AndroidGyro(hardwareMap);
        AndroidGyro.instance.start();
        gyro.initAntiDrift();
        waitForStart();
        gyro.startAntiDrift();

        while(opModeIsActive()) {
            telemetry.addData("x", gyro.x);
            telemetry.addData("y", gyro.y);
            telemetry.addData("z", gyro.z);
            telemetry.addData("heading", gyro.getHeading());
            telemetry.update();
        }

    }
}
