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


@TeleOp(name = "servoTest", group = "a")
public class grabTests extends LinearOpMode {

    // Servos servo;
    public Servo svoJewelLift;
    public Servo svoJewelPivot;

    @Override
    public void runOpMode() {
        //  servo = new Servos(hardwareMap);
        svoJewelPivot = hardwareMap.servo.get("svoJewelPivot");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                svoJewelPivot.setPosition(0.35);
            }
            if (gamepad1.b) {
                svoJewelPivot.setPosition(.36);
            }
            if (gamepad1.y) {
                svoJewelPivot.setPosition(.37);
            }
            if (gamepad1.x) {
                svoJewelPivot.setPosition(0.38);
            }
            if (gamepad1.dpad_up) {
                svoJewelPivot.setPosition(0.77);
            }
            if (gamepad1.dpad_left) {
                svoJewelPivot.setPosition(.76);
            }
            if (gamepad1.dpad_down) {
                svoJewelPivot.setPosition(0.75);
            }
            if (gamepad1.dpad_right) {
                svoJewelPivot.setPosition(.74);
            }
            if (gamepad1.right_bumper) {
                svoJewelPivot.setPosition(.73);
            }
            if (gamepad1.right_trigger > .5) {
                svoJewelPivot.setPosition(.72);
            }
            if (gamepad1.left_bumper) {
                svoJewelPivot.setPosition(.71);
            }
            if (gamepad1.left_trigger > .5) {
                svoJewelPivot.setPosition(.7);
            }


            if (gamepad2.a) {
                svoJewelPivot.setPosition(.69);
            }
            if (gamepad2.b) {
                svoJewelPivot.setPosition(.68); //auto init .48
            }
            if (gamepad2.y) {
                svoJewelPivot.setPosition(.67);
            }
            if (gamepad2.x) {
                svoJewelPivot.setPosition(.66);
            }
            if (gamepad2.dpad_up) {
                svoJewelPivot.setPosition(.65);
            }
            if (gamepad2.dpad_left) {
                svoJewelPivot.setPosition(.64);
            }
            if (gamepad2.dpad_down) {
                svoJewelPivot.setPosition(.63);
            }
            if (gamepad2.dpad_right) {
                svoJewelPivot.setPosition(.62);
            }
            if (gamepad2.right_bumper) {
                svoJewelPivot.setPosition(.61);
            }
            if (gamepad2.right_trigger > .5) {
                svoJewelPivot.setPosition(.6);
            }
            if (gamepad2.left_bumper) {
                svoJewelPivot.setPosition(.59);
            }
            if (gamepad2.left_trigger > .5) {
                svoJewelPivot.setPosition(.58); //.58
            }
        }


    }
}
