package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

    ElapsedTime liftPidTimer;

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

    boolean pivotWatch = false;
    boolean isPivoted = false;

    boolean relicClampWatch = false;
    boolean isRelicClamped = false;

    int liftPidState = 0; // 0 disabled/teleop 1 waiting 2 active
    int liftPidTarget = 0;

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

        liftPidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
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

        if (gamepad1.right_trigger > .5) {
            mtrIntakeLeft.setPower(1);
            mtrIntakeRight.setPower(1);
        } else if (gamepad1.right_bumper) {
            mtrIntakeLeft.setPower(-1);
            mtrIntakeRight.setPower(-1);
        } else {
            mtrIntakeLeft.setPower(0);
            mtrIntakeRight.setPower(0);
        }


        telemetry.addData("leftCurrent", mtrIntakeLeft.getCurrentDraw());
        telemetry.addData("rightCurrent", mtrIntakeRight.getCurrentDraw());

        // ----- relic ----- toggle -----

        if (gamepad2.right_stick_y == 0)
            mtrRelic.setPower(.2);
        else
            mtrRelic.setPower(gamepad2.right_stick_y);

       /*  if (gamepad2.right_trigger > 0.5)
            servos.setRelicGrab(false);
        else if (gamepad2.right_bumper)
            servos.setRelicGrab(true);

        if (gamepad2.left_bumper)
            servos.setRelicPivotGrab(true);
        else if (gamepad2.left_trigger > .5)
            servos.setRelicPivotGrab(false);*/

        if (gamepad2.right_bumper && !relicClampWatch) {
            isRelicClamped = !isRelicClamped;
        }
        if (gamepad2.left_bumper && !pivotWatch) {
            isPivoted = !isPivoted;
        }

        servos.setRelicPivotGrab(pivotWatch);
        servos.setRelicGrab(relicClampWatch);

        pivotWatch = gamepad2.left_bumper;
        relicClampWatch = gamepad2.right_bumper;

        // ----- glyphs -----

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

        // ----- glyph lift -----

        // teleop control is being enacted; disable pid auto-hold
        if (gamepad2.left_stick_y != 0) {
            mtrGlyphLift.setPower(gamepad2.left_stick_y);
            liftPidState = 0;
        }

        switch (liftPidState) {
            case 0:
                // when the gamepad is released, we switch into the waiting state to let the motor
                // move to its rough target
                if (gamepad2.left_stick_y == 0.0) {
                    liftPidTimer.reset();
                    liftPidState = 1;
                }
                break;
            case 1:
                // after the waiting state has elapsed, set our new encoder target and do a position function ig
                if (liftPidTimer.milliseconds() >= 100) {
                    liftPidState = 2;
                    liftPidTarget = mtrGlyphLift.getCurrentPosition();
                }
                break;
            case 2:
                double error = liftPidTarget - mtrGlyphLift.getCurrentPosition();
                mtrGlyphLift.setPower(Range.clip(error * 0.001, -1, 1));
                break;
        }

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
