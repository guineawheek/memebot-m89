package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;

public class Turn {HardwareMap hardMap;
    Telemetry tele;

    public DcMotor rightFront = null;
    public DcMotor leftFront = null;
    public DcMotor rightRear = null;
    public DcMotor leftRear = null;

    static final double     COUNTS_PER_MOTOR_REV = 498;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    public Turn(HardwareMap hMap, Telemetry telemetry) {
        tele = telemetry;
        hardMap= hMap;


        rightFront= hardMap.get(DcMotor.class, "m2");
        leftFront= hardMap.get(DcMotor.class, "m1");
        rightRear= hardMap.get(DcMotor.class, "m3");
        leftRear= hardMap.get(DcMotor.class, "m4");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
    }

    public void SetEncoderOff() {
        //rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void SetEncoderMode() {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void MoveTo(double distance, double degree, double power) {
            double degreeRad = Math.toRadians(degree); // Convert to radians
        double cs = Math.cos(degreeRad);
        double sn = Math.sin(degreeRad);
        // make sure power less than 1 below
        if (power > 0.7) {
            power = 0.7;
        }
        double rightFrontPower = power*(-sn+cs);
        double leftFrontPower = power*(sn+cs);
        double rightRearPower = power*(sn+cs);
        double leftRearPower = power*(-sn+cs);

        SetEncoderMode();
        double targetCounts = (int) (distance * COUNTS_PER_INCH);

        int rightFrontStartPos = rightFront.getCurrentPosition();
        int rightRearStartPos = rightRear.getCurrentPosition();
        int leftFrontStartPos = leftFront.getCurrentPosition();
        int leftRearStartPos = leftRear.getCurrentPosition();

        int target = (int) (distance * COUNTS_PER_INCH);

        int rightFrontEndPos = rightFrontStartPos + (int) (target*(-sn+cs));
        int leftFrontEndPos = leftFrontStartPos +(int) (target*(sn+cs));
        int rightRearEndPos = rightRearStartPos + (int) (target*(sn+cs));
        int leftRearEndPos = leftRearStartPos +(int) (target*(-sn+cs));

        rightFront.setPower(rightFrontPower);
        leftFront.setPower(leftFrontPower);
        rightRear.setPower(rightRearPower);
        leftRear.setPower(leftRearPower);

        rightFront.setTargetPosition(rightFrontEndPos);
        leftFront.setTargetPosition(leftFrontEndPos);
        rightRear.setTargetPosition(rightRearEndPos);
        leftRear.setTargetPosition(leftRearEndPos);
        // run until the end of the match (driver presses STOP)
        while (rightFront.isBusy() || leftFront.isBusy() || rightRear.isBusy() || leftRear.isBusy()) {
            //keep running
            tele.addData("Right Front count",  rightFront.getCurrentPosition());
            tele.addData("Left Front count",  leftFront.getCurrentPosition());
            tele.update();
        }
        StopMotors();
    }
    public void Turn(double TurnDegree, Gyro gyro) {
        // clock is negative; anti-clock positive degree
        // Maximum degree is 180

        if (TurnDegree > 180){
            TurnDegree = 180;
        }
        if (TurnDegree < -180){
            TurnDegree = -180;
        }

        double MaxPower = 0.5;
        double minPower = 0.2;

        double correctionDegree = 4 ;
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
        if (TurnDegree < 0 ) {
            correctionDegree = - correctionDegree;
        }
        target = beginDegree + TurnDegree - correctionDegree;

        runtime.reset();
        runtime.startTime();

        angleDiff=TurnDegree;
        while ( abs(angleDiff) > 1 && runtime.seconds()< maxTime ) {
            double leftPower;
            double rightPower;
            currentDegree =gyro.getZDegree();
            if (TurnDegree>0 ) {
                if (currentDegree <-90) {
                    currentDegree = 360 + currentDegree;
                }
            }
            if (TurnDegree <0 ) {
                if (currentDegree > 90) {
                    currentDegree = -360 + currentDegree;
                }
            }

            angleDiff = (currentDegree - target);
            double drive;
            drive = (angleDiff)/100.0;

            if (abs(drive)> MaxPower ){
                drive = MaxPower * abs(drive)/drive;
            }
            if (abs(drive)< minPower ){
                if (drive>0) {
                    drive = minPower;
                }
                else if(drive<0){
                    drive =-minPower;
                }
                else {
                    drive = 0;
                }
            }

            leftPower    = Range.clip(drive, -1.0, 1.0) ;
            rightPower   = Range.clip(-drive, -1.0, 1.0) ;
            leftFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightRear.setPower(rightPower);

            tele.addData("Left Power",  leftPower);
            tele.addData("right Power",  rightPower);
            tele.addData("beginDegree", beginDegree);
            tele.addData("CurrentDegree",currentDegree);
            tele.addData("angleDiff",angleDiff);
            tele.update();
        }
        StopMotors();

        tele.addData("Loop Done-Angle",gyro.getZDegree());
        tele.update();
    }

    public void MoveToByTime(double time , double direction , double power){

    }
    public void StopMotors(){
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
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
            leftFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightRear.setPower(rightPower);

            tele.addData("Left Power",  leftPower);
            tele.addData("right Power",  rightPower);
            tele.addData("beginDegree", beginDegree);
            tele.addData("Degree",currentDeg);
            tele.addData("angleDiff",angleDiff);
            tele.update();
        }
        StopMotors();

        tele.addData("Loop Done-Angle",gyro.getZDegree());
        tele.update();
    }*/
}
