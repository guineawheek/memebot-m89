package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.openftc.hardware.rev.OpenRevDcMotorImplEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "New Teleop with 2 Servos", group = "new")
public class TeleopBeta extends LinearOpMode {

    private LinearOpMode opMode;
    HolonomicDrivebase drivebase;
    Servos servos;

    OpenRevDcMotorImplEx mtrIntakeLeft;
    OpenRevDcMotorImplEx mtrIntakeRight;

    DcMotor mtrGlyphLift;

    DcMotor mtrRelic;

    public void init_() {

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
    public void runOpMode() throws InterruptedException {

        init_();

        waitForStart();

        boolean ScoringPositionActivated = false;
        boolean isGrabbed = false;
        boolean isPressed = false;
        boolean bumperisPressed = false;

        while (opModeIsActive()) {
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

            if (gamepad2.right_stick_y == 0) {
                mtrRelic.setPower(-.3);
            } else {
                mtrRelic.setPower(gamepad2.right_stick_y);
            }


            // ----- glyphs -----
            mtrGlyphLift.setPower(gamepad2.left_stick_y);

            if (gamepad1.right_trigger > .5 && !isPressed) {
                ScoringPositionActivated = !ScoringPositionActivated;
            }

            servos.setFlipperUp(ScoringPositionActivated);

            if(gamepad1.right_trigger > .5 ){
                isPressed = true;
            }
            else{
                isPressed = false;
            }

            if (gamepad1.right_bumper && !bumperisPressed) {
                isGrabbed = !isGrabbed;
            }
            servos.setFlipperGrab(isGrabbed);

            bumperisPressed = gamepad1.right_bumper;

        }
    }
}
