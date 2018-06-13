package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teleop")
public class ThiccTeleop extends OpMode {
    HolonomicDrivebase drivebase;

    DcMotor mtrIntakeLeft;
    DcMotor mtrIntakeRight;

    DcMotor mtrGlyphLift;

    DcMotor mtrRelic;

    Servo svoJewelPivot;
    Servo svoJewelLift;

    Servo svoRelicPivot;
    Servo svoRelicGrab;

    Servo svoFlipperFlip;
    Servo svoFlipperTop;
    Servo svoFlipperBottom;

    Servo svoAutoLeft;
    Servo svoAutoRight;

    @Override
    public void init() {
        drivebase = new HolonomicDrivebase(hardwareMap);

        mtrIntakeLeft = hardwareMap.dcMotor.get("mtrIntakeLeft");
        mtrIntakeRight = hardwareMap.dcMotor.get("mtrIntakeRight");

        mtrGlyphLift = hardwareMap.dcMotor.get("mtrGlyphLift");
        mtrRelic = hardwareMap.dcMotor.get("mtrRelic");

        svoAutoLeft = hardwareMap.servo.get("svoAutoLeft");
        svoAutoRight = hardwareMap.servo.get("svoAutoRight");

        svoJewelLift = hardwareMap.servo.get("svoJewelLift");
        svoJewelPivot = hardwareMap.servo.get("svoJewelPivot");

        svoFlipperBottom = hardwareMap.servo.get("svoFlipperBottom");
        svoFlipperFlip = hardwareMap.servo.get("svoFlipperFlip");

        svoFlipperTop = hardwareMap.servo.get("svoFlipperTop");
        svoRelicGrab = hardwareMap.servo.get("svoRelicGrab");
        svoRelicPivot = hardwareMap.servo.get("svoRelicPivot");

    }

    @Override
    public void loop() {

        // ----- drive code -----
        // square inputs; this makes smaller movements easier
        double x1 = Math.copySign(Math.pow(gamepad1.left_stick_x, 2), gamepad1.left_stick_x);
        double y1 = Math.copySign(Math.pow(gamepad1.left_stick_y, 2), -gamepad1.left_stick_y);
        double x2 = Math.copySign(Math.pow(gamepad1.right_stick_x, 2), gamepad1.right_stick_x);

        drivebase.driveArcade(x1, y1, x2, 1);


        // ----- intake ------
        double intakePower = 0;
        if (gamepad1.right_bumper) // intake in
            intakePower = 1;
        else if (gamepad1.right_trigger > 0.5) // intake reverse
            intakePower = -1;

        mtrIntakeLeft.setPower(intakePower);
        mtrIntakeRight.setPower(intakePower);


    }
}
