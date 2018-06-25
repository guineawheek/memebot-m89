package org.firstinspires.ftc.teamcode.function_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Gyro;
import org.firstinspires.ftc.teamcode.subsystems.Servos;

@Disabled
@TeleOp(name = "servoTest", group = "a")
public class grabTests extends LinearOpMode {

    // Servos servo;
    public Servo svoFlipperFlip;


    @Override
    public void runOpMode() {
        //  servo = new Servos(hardwareMap);
        svoFlipperFlip = hardwareMap.servo.get("svoFlipperFlip");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                svoFlipperFlip.setPosition(0.54);
            }
            if (gamepad1.b) {
                svoFlipperFlip.setPosition(.55);
            }
            if (gamepad1.y) {
                svoFlipperFlip.setPosition(.56);
            }
            if (gamepad1.x) {
                svoFlipperFlip.setPosition(.57);
            }
            if (gamepad1.dpad_up) {
                svoFlipperFlip.setPosition(.58); // resting
            }
            if (gamepad1.dpad_left) {
                svoFlipperFlip.setPosition(.59);
            }
            if (gamepad1.dpad_down) {
                svoFlipperFlip.setPosition(.6);
            }
            if (gamepad1.dpad_right) {
                svoFlipperFlip.setPosition(.06);
            }
            if (gamepad1.right_bumper) {
                svoFlipperFlip.setPosition(.07);
            }
            if (gamepad1.right_trigger > .5) {
                svoFlipperFlip.setPosition(.08);
            }
            if (gamepad1.left_bumper) {
                svoFlipperFlip.setPosition(.09);
            }
            if (gamepad1.left_trigger > .5) {
                svoFlipperFlip.setPosition(.10);
            }


            if (gamepad2.a) {
                svoFlipperFlip.setPosition(.11); //halfway up
            }
            if (gamepad2.b) {
                svoFlipperFlip.setPosition(.12); //auto init .48
            }
            if (gamepad2.y) {
                svoFlipperFlip.setPosition(.13);
            }
            if (gamepad2.x) {
                svoFlipperFlip.setPosition(.14);
            }
            if (gamepad2.dpad_up) {
                svoFlipperFlip.setPosition(.15);
            }
            if (gamepad2.dpad_left) {
                svoFlipperFlip.setPosition(.16);
            }
            if (gamepad2.dpad_down) {
                svoFlipperFlip.setPosition(.17);
            }
            if (gamepad2.dpad_right) {
                svoFlipperFlip.setPosition(.18);
            }
            if (gamepad2.right_bumper) {
                svoFlipperFlip.setPosition(.19);
            }
            if (gamepad2.right_trigger > .5) {
                svoFlipperFlip.setPosition(.20);
            }
            if (gamepad2.left_bumper) {
                svoFlipperFlip.setPosition(.21);
            }
            if (gamepad2.left_trigger > .5) {
                svoFlipperFlip.setPosition(.22); //.58
            }
        }


    }
}
