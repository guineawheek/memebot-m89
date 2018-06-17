package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Jewel {

    HardwareMap HardwareMap;
    Telemetry telemetry;

    Servo svoJewelLift;
    Servo svoJewelPivot;

    ColorSensor snsColorSensorLeft;
    ColorSensor snsColorSensorRight;

    //all constants need testing

    public static final double DOWN_STOW_POS = .1171875; //ready for init
    public static final double SWING_STOW_POS = .33203125;

    public static final double DOWN_EX_POS = .83984375; //looking for the correct ball
    public static final double SWING_EX_POS = .60546875;

    public static final double SWING_LEFT = .390625; //swing left or right to knock the jewel off
    public static final double SWING_RIGHT = .78125;

    public Jewel(HardwareMap hardwareMap, Telemetry telemetry) {
        this.HardwareMap = hardwareMap;
        this.telemetry = telemetry;

        svoJewelLift = hardwareMap.servo.get("svoJewelLift");
        svoJewelPivot = hardwareMap.servo.get("svoJewelPivot");

        snsColorSensorLeft = hardwareMap.colorSensor.get("snsColorSensorLeft");
        snsColorSensorRight = hardwareMap.colorSensor.get("snsColorSensorRight");
    }

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

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}