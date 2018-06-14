package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.openftc.hardware.rev.OpenRevDcMotorImplEx;

@TeleOp(name = "Teleop")
public class ThiccTeleop extends OpMode {
    HolonomicDrivebase drivebase;
    Servos servos;

    OpenRevDcMotorImplEx mtrIntakeLeft;
    OpenRevDcMotorImplEx mtrIntakeRight;

    DcMotor mtrGlyphLift;

    DcMotor mtrRelic;

    @Override
    public void init() {
        drivebase = new HolonomicDrivebase(hardwareMap);
        servos = new Servos(hardwareMap);
        servos.init();

        mtrIntakeLeft = new OpenRevDcMotorImplEx((DcMotorImplEx) hardwareMap.dcMotor.get("mtrIntakeLeft"));
        mtrIntakeRight = new OpenRevDcMotorImplEx((DcMotorImplEx) hardwareMap.dcMotor.get("mtrIntakeRight"));

        mtrGlyphLift = hardwareMap.dcMotor.get("mtrGlyphLift");
        mtrRelic = hardwareMap.dcMotor.get("mtrRelic");

        mtrGlyphLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrRelic.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        mtrIntakeLeft.setPower(gamepad1.left_bumper ? -1 : gamepad1.left_trigger);
        mtrIntakeRight.setPower(gamepad1.right_bumper ? -1 : gamepad1.right_trigger);
        telemetry.addData("leftCurrent", mtrIntakeLeft.getCurrentDraw());
        telemetry.addData("rightCurrent", mtrIntakeRight.getCurrentDraw());

        // ----- relic -----
        mtrRelic.setPower(gamepad2.right_stick_y);

        // ----- glyphs -----
        mtrGlyphLift.setPower(gamepad2.left_stick_y);
    }
}
