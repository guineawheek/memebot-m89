package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;

//port 1 switch sensor
public class DriveTrain {
    HardwareMap hardMap;
    Telemetry tele;

    public DcMotor mtrFR = null;
    public DcMotor mtrFL = null;
    public DcMotor mtrBR = null;
    public DcMotor mtrBL = null;

    ModernRoboticsI2cRangeSensor range;

    static final double COUNTS_PER_MOTOR_REV = 498;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    public DriveTrain(HardwareMap hMap, Telemetry telemetry) {
        tele = telemetry;
        hardMap = hMap;


        mtrFR = hardMap.get(DcMotor.class, "m2");
        mtrFL = hardMap.get(DcMotor.class, "m1");
        mtrBR = hardMap.get(DcMotor.class, "m3");
        mtrBL = hardMap.get(DcMotor.class, "m4");

        mtrFR.setDirection(DcMotor.Direction.REVERSE);
        mtrFL.setDirection(DcMotor.Direction.FORWARD);
        mtrBR.setDirection(DcMotor.Direction.REVERSE);
        mtrBL.setDirection(DcMotor.Direction.FORWARD);

        range = hardMap.get(ModernRoboticsI2cRangeSensor.class, "range");
    }

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

    public void MoveTo(double distance, double degree, double power) {
        double degreeRad = Math.toRadians(degree); // Convert to radians
        double cs = Math.cos(degreeRad);
        double sn = Math.sin(degreeRad);
        // make sure power less than 1 below
        if (power > 0.7) {
            power = 0.7;
        }
        double rightFrontPower = power * (-sn + cs);
        double leftFrontPower = power * (sn + cs);
        double rightRearPower = power * (sn + cs);
        double leftRearPower = power * (-sn + cs);

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

        mtrFR.setPower(rightFrontPower);
        mtrFL.setPower(leftFrontPower);
        mtrBR.setPower(rightRearPower);
        mtrBL.setPower(leftRearPower);

        mtrFR.setTargetPosition(rightFrontEndPos);
        mtrFL.setTargetPosition(leftFrontEndPos);
        mtrBR.setTargetPosition(rightRearEndPos);
        mtrBL.setTargetPosition(leftRearEndPos);
        // run until the end of the match (driver presses STOP)
        while (mtrFR.isBusy() || mtrFL.isBusy() || mtrBR.isBusy() || mtrBL.isBusy()) {
            //keep running
            tele.addData("Right Front count", mtrFR.getCurrentPosition());
            tele.addData("Left Front count", mtrFL.getCurrentPosition());
            tele.update();
        }
        stopMotors();
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
        while (abs(angleDiff) > 1 && runtime.seconds() < maxTime) {
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

            tele.addData("Left Power", leftPower);
            tele.addData("right Power", rightPower);
            tele.addData("beginDegree", beginDegree);
            tele.addData("CurrentDegree", currentDegree);
            tele.addData("angleDiff", angleDiff);
            tele.update();
        }
        stopMotors();

        tele.addData("Loop Done-Angle", gyro.getZDegree());
        tele.update();
    }

    public void MoveToByTime(long time, double direction, double power) {
        double degreeRad = Math.toRadians(direction); // Convert to radians
        double cs = Math.cos(degreeRad);
        double sn = Math.sin(degreeRad);

        double fr = power * (-sn + cs);
        double fl = power * (sn + cs);
        double br = power * (sn + cs);
        double bl = power * (-sn + cs);

        mtrFL.setPower(fl);
        mtrFR.setPower(fr);
        mtrBL.setPower(bl);
        mtrBR.setPower(br);

        sleep(time);

        stopMotors();
    }

    public void MoveToByRange(double distance, double direction, double power) {
        double degreeRad = Math.toRadians(direction); // Convert to radians
        double cs = Math.cos(degreeRad);
        double sn = Math.sin(degreeRad);

        double fr = power * (-sn + cs);
        double fl = power * (sn + cs);
        double br = power * (sn + cs);
        double bl = power * (-sn + cs);

        while (range.cmUltrasonic() > distance) {
            mtrFL.setPower(fl);
            mtrFR.setPower(fr);
            mtrBL.setPower(bl);
            mtrBR.setPower(br);
        }

        stopMotors();
    }

    public void MoveToBySwitch(double direction, double power) {
        double degreeRad = Math.toRadians(direction); // Convert to radians
        double cs = Math.cos(degreeRad);
        double sn = Math.sin(degreeRad);

        double fr = power * (-sn + cs);
        double fl = power * (sn + cs);
        double br = power * (sn + cs);
        double bl = power * (-sn + cs);
        double a = 0; //NOTICE - NOT CORRECT YET! DO NOT USE; MAKE SURE TO CORRECT IT OR ELSE THE WORLD WILL END !!!!!!!!
        while (a == 0){
            mtrFL.setPower(fl);
            mtrFR.setPower(fr);
            mtrBL.setPower(bl);
            mtrBR.setPower(br);
        }

        stopMotors();
    }

    public void stopMotors() {
        mtrFR.setPower(0);
        mtrFL.setPower(0);
        mtrBR.setPower(0);
        mtrBL.setPower(0);
    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    /*public void Turn_old(double TurnDegree, Gyro gyro) {
        // clock is negative; anti-clock positive degree
        // Maximum degree is 180
        if (TurnDegree > 180){
            TurnDegree = 180;
        }
        if (TurnDegree < -180){
            TurnDegree = -180;
        }

        double MaxPower = 0.5;
        double correctDegree = 0;
        Orientation angles;
        double beginDegree;
        double target;
        double angleDiff;

        SetEncoderOff();
        gyro.ResetAngle();

        beginDegree = gyro.getZDegree();

        if (TurnDegree < 0 ) {
            correctDegree = - correctDegree;
        }
        target = beginDegree + TurnDegree - correctDegree;

        angleDiff=TurnDegree;
        while ( abs(angleDiff) > 1 ) {
            double leftPower;
            double rightPower;
            double currentDeg =gyro.getZDegree();
            angleDiff = (currentDeg - target);
            double drive;
            drive = (angleDiff)/100.0;

            if (abs(drive)> MaxPower ){
                drive = MaxPower * abs(drive)/drive;
            }

            leftPower    = Range.clip(drive, -1.0, 1.0) ;
            rightPower   = Range.clip(-drive, -1.0, 1.0) ;
            mtrFL.setPower(leftPower);
            mtrBL.setPower(leftPower);
            mtrFR.setPower(rightPower);
            mtrBR.setPower(rightPower);

            tele.addData("Left Power",  leftPower);
            tele.addData("right Power",  rightPower);
            tele.addData("beginDegree", beginDegree);
            tele.addData("Degree",currentDeg);
            tele.addData("angleDiff",angleDiff);
            tele.update();
        }
        stopMotors();

        tele.addData("Loop Done-Angle",gyro.getZDegree());
        tele.update();
    }*/
}
