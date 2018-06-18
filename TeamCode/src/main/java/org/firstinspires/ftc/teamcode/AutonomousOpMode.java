package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Gyro;
import org.firstinspires.ftc.teamcode.subsystems.Jewel;
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.VuMarkRecognition;

import static java.lang.Math.abs;

public abstract class AutonomousOpMode extends LinearOpMode {

    DriveTrain motors;
    Gyro gyro = null;
    Jewel jewel;
    Servos servos;
    VuMarkRecognition vuMark;
    ModernRoboticsI2cRangeSensor snsRange;
    DigitalChannel snsLimitSwitch; //port 1

    DcMotor mtrFR;
    DcMotor mtrFL;
    DcMotor mtrBR;
    DcMotor mtrBL;

    Servo svoJewelLift;
    Servo svoJewelPivot;

    ColorSensor snsColorSensorLeft;
    ColorSensor snsColorSensorRight;

    static final double COUNTS_PER_MOTOR_REV = 498;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


//jewel
    public static final double DOWN_STOW_POS = .1171875; //ready for init
    public static final double SWING_STOW_POS = .33203125;

    public static final double DOWN_EX_POS = .83984375; //looking for the correct ball
    public static final double SWING_EX_POS = .60546875;

    public static final double SWING_LEFT = .390625; //swing left or right to knock the jewel off
    public static final double SWING_RIGHT = .78125;

    public void initit() {

        mtrFR = hardwareMap.get(DcMotor.class, "m2");
        mtrFL = hardwareMap.get(DcMotor.class, "m1");
        mtrBR = hardwareMap.get(DcMotor.class, "m3");
        mtrBL = hardwareMap.get(DcMotor.class, "m4");

        svoJewelLift = hardwareMap.servo.get("svoJewelLift");
        svoJewelPivot = hardwareMap.servo.get("svoJewelPivot");

        snsColorSensorLeft = hardwareMap.colorSensor.get("snsColorSensorLeft");
        snsColorSensorRight = hardwareMap.colorSensor.get("snsColorSensorRight");

        mtrFR.setDirection(DcMotor.Direction.REVERSE);
        mtrFL.setDirection(DcMotor.Direction.FORWARD);
        mtrBR.setDirection(DcMotor.Direction.REVERSE);
        mtrBL.setDirection(DcMotor.Direction.FORWARD);

        motors = new DriveTrain(hardwareMap, telemetry);
        gyro = new Gyro(hardwareMap, telemetry);
        jewel = new Jewel(hardwareMap, telemetry);
        servos = new Servos(hardwareMap);

        snsRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "snsRange");
        snsLimitSwitch = hardwareMap.get(DigitalChannel.class, "snsLimitSwitch");

        vuMark = new VuMarkRecognition(this.hardwareMap, this.telemetry);

        servos.autoInit();

        jewel.jewelStow();

        telemetry.addData("Status", "DriveTrain Initialized");
        telemetry.update();
    }

    /*
    public void moveBySensor(double arg1, double arg2) {
        while(!sensor.isDone() && opModeIsActive()) {
            motors.moveTo(arg1, arg2);
        }
    }
     */
    public void SetEncoderOff() {
        //mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void SetEncoderMode() {
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void MoveToByTime(long time, double direction, double power) {
        motors.MoveTo(direction, power);
        sleep(time);
        motors.stopMotors();
    }

    public void MoveToByRange(double distance, double direction, double power) {
        motors.MoveTo(direction,power);
        while (snsRange.cmUltrasonic() > distance && opModeIsActive()) {
            telemetry.addData("snsRange", snsRange.cmUltrasonic());
        }
        motors.stopMotors();
    }

    public void MoveToBySwitch(double direction, double power) {
        SetEncoderOff();
        motors.MoveTo(direction,power);
        while (snsLimitSwitch.getState() && opModeIsActive()) {
        }
        motors.stopMotors();

    }

    public void MoveToByEncoder(double distance, double degree, double power) {
        double degreeRad = Math.toRadians(degree); // Convert to radians
        double cs = Math.cos(degreeRad);
        double sn = Math.sin(degreeRad);


        //SetEncoderOff();
        SetEncoderMode();
        double targetCounts = (int) (distance * COUNTS_PER_INCH);

        int rightFrontStartPos = mtrFR.getCurrentPosition();
        int rightRearStartPos = mtrBR.getCurrentPosition();
        int leftFrontStartPos = mtrFL.getCurrentPosition();
        int leftRearStartPos = mtrBL.getCurrentPosition();

        int target = (int) (distance * COUNTS_PER_INCH);

        int rightFrontEndPos = rightFrontStartPos + (int) (target * (-sn + cs));
        int leftFrontEndPos = leftFrontStartPos + (int) (target * (sn + cs));
        int rightRearEndPos = rightRearStartPos + (int) (target * (sn + cs));
        int leftRearEndPos = leftRearStartPos + (int) (target * (-sn + cs));

        /*
        boolean frontLeft = (int) (target * (sn + cs)) != 0;
        DcMotor enc = frontLeft? mtrFL : mtrFR;
        int end = frontLeft ? leftFrontEndPos : rightFrontEndPos;
        while (Math.abs(enc.getCurrentPosition() - end) > 3 && opModeIsActive()) {
            double pwr = Range.clip((enc.getCurrentPosition() - end) * 0.002 * power, -power, power);

            telemetry.addData("value", enc.getCurrentPosition());
            telemetry.addData("target", end);
            telemetry.addData("dist", enc.getCurrentPosition() - end);
            telemetry.update();
        }
        motors.stopMotors();
        */
        double pwr = power;
        double rightFrontPower = pwr * (-sn + cs);
        double leftFrontPower = pwr * (sn + cs);
        double rightRearPower = pwr * (sn + cs);
        double leftRearPower = pwr * (-sn + cs);

        mtrFR.setPower(rightFrontPower);
        mtrFL.setPower(leftFrontPower);
        mtrBR.setPower(rightRearPower);
        mtrBL.setPower(leftRearPower);

        mtrFR.setTargetPosition(rightFrontEndPos);
        mtrFL.setTargetPosition(leftFrontEndPos);
        mtrBR.setTargetPosition(rightRearEndPos);
        mtrBL.setTargetPosition(leftRearEndPos);
        // run until the end of the match (driver presses STOP)
        //while ((Math.abs(mtrFL.getCurrentPosition() - end) > 100 )&& opModeIsActive()) {}
        while (mtrFL.isBusy() && opModeIsActive()) {}
        /*|| mtrFL.isBusy() || mtrBR.isBusy() || mtrBL.isBusy())*/
        motors.stopMotors();

    }
    public void Turn(double TurnDegree, Gyro gyro) {
        // clock is negative; anti-clock positive degree
        // Maximum degree is 180

        if (TurnDegree > 180) {
            TurnDegree = 180;
        }
        if (TurnDegree < -180) {
            TurnDegree = -180;
        }

        double MaxPower = 0.5;
        double minPower = 0.2;

        double correctionDegree = 4;
        Orientation angles;
        double beginDegree;
        double currentDegree;
        double target;
        double angleDiff;
        double maxTime = 6; //seconds
        ElapsedTime runtime = new ElapsedTime();

        SetEncoderOff();
        gyro.ResetAngle();

        beginDegree = gyro.getZDegree();
        if (TurnDegree < 0) {
            correctionDegree = -correctionDegree;
        }
        target = beginDegree + TurnDegree - correctionDegree;

        runtime.reset();
        runtime.startTime();

        angleDiff = TurnDegree;
        while (abs(angleDiff) > 1 && runtime.seconds() < maxTime && opModeIsActive()) {
            double leftPower;
            double rightPower;
            currentDegree = gyro.getZDegree();
            if (TurnDegree > 0) {
                if (currentDegree < -90) {
                    currentDegree = 360 + currentDegree;
                }
            }
            if (TurnDegree < 0) {
                if (currentDegree > 90) {
                    currentDegree = -360 + currentDegree;
                }
            }

            angleDiff = (currentDegree - target);
            double drive;
            drive = (angleDiff) / 100.0;

            if (abs(drive) > MaxPower) {
                drive = MaxPower * abs(drive) / drive;
            }
            if (abs(drive) < minPower) {
                if (drive > 0) {
                    drive = minPower;
                } else if (drive < 0) {
                    drive = -minPower;
                } else {
                    drive = 0;
                }
            }

            leftPower = Range.clip(drive, -1.0, 1.0);
            rightPower = Range.clip(-drive, -1.0, 1.0);
            mtrFL.setPower(leftPower);
            mtrBL.setPower(leftPower);
            mtrFR.setPower(rightPower);
            mtrBR.setPower(rightPower);

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("right Power", rightPower);
            telemetry.addData("beginDegree", beginDegree);
            telemetry.addData("CurrentDegree", currentDegree);
            telemetry.addData("angleDiff", angleDiff);
            telemetry.update();
        }
        motors.stopMotors();

        telemetry.addData("Loop Done-Angle", gyro.getZDegree());
        telemetry.update();
    }

   //jewel


    public void ledOn(boolean x) {
        snsColorSensorLeft.enableLed(x);
        snsColorSensorRight.enableLed(x);
    }

    public void jewelStow() {
        svoJewelLift.setPosition(DOWN_STOW_POS);
        svoJewelPivot.setPosition(SWING_STOW_POS);
    }

    public void jewelExplore() {
        svoJewelPivot.setPosition(SWING_EX_POS);
        svoJewelLift.setPosition(DOWN_EX_POS);
    }

    public void swingL() {
        svoJewelPivot.setPosition(SWING_LEFT);
    }

    public void swingR() {
        svoJewelPivot.setPosition(SWING_RIGHT);
    }

    public void jewelAction(boolean lb, boolean lr, boolean rb, boolean rr) { //red version (if we want blue, rb,rr,lb,lr)
        if (lb && lr && rb && rr) {          // color sensors all fail
            swingL();
            swingR();
            sleepC(100);
        } else if (lb && lr) { //left sensor fails
            if (rb) {
                swingL(); //   red-blue
                sleepC(100);
            } else {
                swingR(); //    blue-red
                sleepC(100);
            }
        } else if (rb && rr) { //right sensor fails
            if (lb) {
                swingR(); //     blue-red
                sleepC(100);
            } else {
                swingL(); //     red-blue
                sleepC(100);
            }
        } else if ((rb && lb) || (lr && rr)) { //sensors both working but giving opposite readings
            // do nothing
        } else if (rb) { //both sensors are reading
            swingL();//     red-blue
            sleepC(100);
        } else {
            swingR(); //       blue-red
            sleepC(100);
        }
    }

    public boolean rr() {
        sleepC(10);
        return snsColorSensorRight.red() >= snsColorSensorRight.blue();
    }

    public boolean lr() {
        sleepC(10);
        return snsColorSensorLeft.red() >= snsColorSensorLeft.blue();
    }

    public boolean rb() {
        sleepC(10);
        return snsColorSensorRight.red() <= snsColorSensorRight.blue();
    }

    public boolean lb() {
        sleepC(10);
        return snsColorSensorLeft.blue() >= snsColorSensorLeft.red();
    }

    public void sleepC(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void hitBallsRedisTru(boolean color) {
        if (color) {
            jewelAction(lb(), lr(), rb(), rr());
        } else {
            jewelAction(rb(), rr(), lb(), lr());
        }
    }
    public void hitRedJewel() {
        ledOn(true);
        jewelExplore();
        sleep(1000);
        hitBallsRedisTru(true);
        jewelStow();
    }

    public void hitBlueJewel() {
        ledOn(true);
        jewelExplore();
        sleep(1000);
        hitBallsRedisTru(false);
        jewelStow();
    }

}
