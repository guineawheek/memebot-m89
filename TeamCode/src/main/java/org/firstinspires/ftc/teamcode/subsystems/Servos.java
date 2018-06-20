package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Servos {
    /*

back0: lift
back1: relic
back2/3: intakes

servos:
back0: jewelpivot (0.57 center)
back1: placeleft
back5: placeright

front0: relicupdown (0 up 0.58 down)
front1: flipgripback (0.57/0.15...?)
front2: flipper (0.77 down, 0.26 up)
front3: relicgrab 0.57/0.87 grab
front4: flipgripfront 0.57/0.1 (0.3?) grab
front5: jeweldown 0.23
     */

    public static final double JEWEL_PIVOT_CENTER = 0.57;
    public static final double JEWEL_LIFT_UP = 0.23;

    public static final double RELIC_PIVOT_INIT = 1;
    public static final double RELIC_PIVOT_UP = 0;
    public static final double RELIC_PIVOT_GRAB = 0.6;

    public static final double RELIC_UNGRAB = 0.57;
    public static final double RELIC_GRAB = 1.0;

    public static final double FLIPPER_UP = 0;
    public static final double FLIPPER_DOWN = 0.555;

    public static final double FLIPPER_UNGRAB = 0.57;
    public static final double FLIPPER_GRAB = 0.15;

    public static final double ALIGN_DOWN = 0; //need to test
    public static final double ALIGN_UP = 0.55;

    public static final double FLIPPER_UNGRAB_RIGHT = .65;
    public static final double FLIPPER_GRAB_RIGHT = .85;

    public static final double FLIPPER_UNGRAB_LEFT = .4;
    public static final double FLIPPER_GRAB_LEFT = .1;

    Servo svoJewelPivot;
    Servo svoJewelLift;

    Servo svoRelicPivot;
    Servo svoRelicGrab;

    Servo svoFlipperFlip;
 //   Servo svoFlipperTop;
 //   Servo svoFlipperBottom;

    Servo svoFlipperLeft;
    Servo svoFlipperRight;

    Servo svoAutoAlign;




    public Servos(HardwareMap hardwareMap) {
        svoAutoAlign = hardwareMap.servo.get("svoAutoAlign");

        svoJewelLift = hardwareMap.servo.get("svoJewelLift");
        svoJewelPivot = hardwareMap.servo.get("svoJewelPivot");

        //svoFlipperBottom = hardwareMap.servo.get("svoFlipperBottom");
        //svoFlipperTop = hardwareMap.servo.get("svoFlipperTop");
        svoFlipperFlip = hardwareMap.servo.get("svoFlipperFlip");

        svoFlipperLeft = hardwareMap.servo.get("svoFlipperLeft");
        svoFlipperRight = hardwareMap.servo.get("svoFlipperRight");

        svoRelicGrab = hardwareMap.servo.get("svoRelicGrab");
        svoRelicPivot = hardwareMap.servo.get("svoRelicPivot");

    }

    public void init() {

        setRelicPivotGrab(false); // unworkable for auton, here as a temp
        setRelicGrab(false);
       // svoJewelPivot.setPosition(JEWEL_PIVOT_CENTER);
        stowJewels();
        setJewelLiftUp(true);
        setFlipperUp(false);

    }
    public void autoInit(){
        svoRelicPivot.setPosition(RELIC_PIVOT_INIT);
        setRelicGrab(true);
        //have jewels already
        //
        setFlipperUp(false);
        setFlipperGrab(true);
        setAutoAlign(false);
    }

    public void setJewelLiftUp(boolean up) {
        /*
        equivalent to:
        if (up)
            svoJewelLift.setPosition(JEWEL_LIFT_UP);
        else
            svoJewelLift.setPosition(0.57);
         */

        svoJewelLift.setPosition(up ? JEWEL_LIFT_UP : 0.57);
    }
    public void stowJewels(){
        svoJewelLift.setPosition(.1171875);
        svoJewelPivot.setPosition(.33203125);
    }

    public void setRelicPivotGrab(boolean isInGrabPosition) {
        svoRelicPivot.setPosition(isInGrabPosition ? RELIC_PIVOT_GRAB : RELIC_PIVOT_UP);
    }

    public void setRelicGrab(boolean isGrabbed) {
        svoRelicGrab.setPosition(isGrabbed ? RELIC_GRAB :RELIC_UNGRAB);
    }

    public void setFlipperUp(boolean up) {
        svoFlipperFlip.setPosition(up ? FLIPPER_UP : FLIPPER_DOWN);
    }

    public void setFlipperGrab(boolean grab) {
        //svoFlipperTop.setPosition(grab ? FLIPPER_GRAB : FLIPPER_UNGRAB);
       // svoFlipperBottom.setPosition(grab ? FLIPPER_GRAB : FLIPPER_UNGRAB);

        svoFlipperLeft.setPosition(grab ? FLIPPER_GRAB_LEFT : FLIPPER_UNGRAB_LEFT);
        svoFlipperRight.setPosition(grab ? FLIPPER_GRAB_RIGHT : FLIPPER_UNGRAB_RIGHT);
    }


    public void setAutoAlign(boolean aligning){
        svoAutoAlign.setPosition(aligning ? ALIGN_UP : ALIGN_DOWN);
    }
}
