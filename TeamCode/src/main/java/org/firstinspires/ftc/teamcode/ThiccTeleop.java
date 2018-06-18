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

    /*
    boolean ScoringPositionActivated = false;
    boolean isGrabbed = false;
    boolean isPressed = false;
    boolean bumperisPressed = false;
    */
    boolean flipWatch = false;
    boolean isFlipped = false;

    boolean grabWatch = false;
    boolean isGrabbed = false;
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
        // square inputs; this makes smaller movements easier - le
        double x1 = Math.copySign(Math.pow(gamepad1.left_stick_x, 1), gamepad1.left_stick_x);
        double y1 = Math.copySign(Math.pow(gamepad1.left_stick_y, 1), -gamepad1.left_stick_y);
        double x2 = Math.copySign(Math.pow(gamepad1.right_stick_x, 1), gamepad1.right_stick_x);

        drivebase.driveArcade(x1, y1, x2, 1);

        // ----- intake ------
        //mtrIntakeLeft.setPower(gamepad1.left_bumper ? -1 : gamepad1.left_trigger);
        //mtrIntakeRight.setPower(gamepad1.right_bumper ? -1 : gamepad1.right_trigger);

        if(gamepad1.right_trigger > .5){
            mtrIntakeLeft.setPower(1);
            mtrIntakeRight.setPower(1);
        }
        else if(gamepad1.right_bumper){
            mtrIntakeLeft.setPower(-1);
            mtrIntakeRight.setPower(-1);
        } else {
            mtrIntakeLeft.setPower(0);
            mtrIntakeRight.setPower(0);
        }


        telemetry.addData("leftCurrent", mtrIntakeLeft.getCurrentDraw());
        telemetry.addData("rightCurrent", mtrIntakeRight.getCurrentDraw());

        // ----- relic -----

        if (gamepad2.right_stick_y == 0)
            mtrRelic.setPower(.2);
        else
            mtrRelic.setPower(gamepad2.right_stick_y);

        if (gamepad2.right_trigger > 0.5)
            servos.setRelicGrab(false);
        else if (gamepad2.right_bumper)
            servos.setRelicGrab(true);

        if (gamepad2.left_bumper)
            servos.setRelicPivotGrab(true);
        else if (gamepad2.left_trigger > .5)
            servos.setRelicPivotGrab(false);
        // ----- glyphs -----

        mtrGlyphLift.setPower(gamepad2.left_stick_y);

        if (gamepad1.left_trigger > 0.5 && !flipWatch) {
            isFlipped = !isFlipped;
            if (isFlipped)
                isGrabbed = true;
        }

        if (gamepad1.left_bumper && !grabWatch)
            isGrabbed = !isGrabbed;

        servos.setFlipperUp(isFlipped);
        servos.setFlipperGrab(isGrabbed);

        flipWatch = gamepad1.left_trigger > 0.5;
        grabWatch = gamepad1.left_bumper;


        /*
      if (gamepad1.left_trigger > .5 && !isPressed)
            ScoringPositionActivated = !ScoringPositionActivated;

       if(ScoringPositionActivated){
           servos.setFlipperGrab(true);//will be true
           servos.setFlipperUp(true);
       }
       else if (!ScoringPositionActivated){
           servos.setFlipperUp(false);
       }
//      servos.setFlipperUp(ScoringPositionActivated);

        if(gamepad1.left_trigger > .5 )
            isPressed = true;
        else
            isPressed = false;

        if (gamepad1.left_bumper && !bumperisPressed)
            isGrabbed = !isGrabbed;

        servos.setFlipperGrab(isGrabbed);

        bumperisPressed = gamepad1.right_bumper;
        */
    }
}
