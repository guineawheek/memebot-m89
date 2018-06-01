package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class HolonomicDrivebase {
    protected DcMotor mtrFL;
    protected DcMotor mtrFR;
    protected DcMotor mtrBL;
    protected DcMotor mtrBR;

    public HolonomicDrivebase(HardwareMap hardwareMap) {
        mtrFL = hardwareMap.dcMotor.get("m1");
        mtrFR = hardwareMap.dcMotor.get("m2");
        mtrBL = hardwareMap.dcMotor.get("m4");
        mtrBR = hardwareMap.dcMotor.get("m3");
    }

    public void driveArcade(double x1, double y1, double x2, double factor) {

        double fl = (x1 + y1 + x2) / factor;
        double fr = (x1 - y1 + x2) / factor;
        double bl = (-x1 + y1 + x2) / factor;
        double br = (-x1 - y1 + x2) / factor;
        double[] powers = {
                Math.abs(fl),
                Math.abs(fr),
                Math.abs(bl),
                Math.abs(br),
        };
        Arrays.sort(powers);

        if (powers[3] > 1) {
            fl /= powers[3];
            fr /= powers[3];
            bl /= powers[3];
            br /= powers[3];
        }
        setDrivePowers(fl, fr, bl, br);
    }

    public void setDrivePowers(double fl, double fr, double bl, double br) {
        mtrFL.setPower(fl);
        mtrFR.setPower(fr);
        mtrBL.setPower(bl);
        mtrBR.setPower(br);
    }

}
