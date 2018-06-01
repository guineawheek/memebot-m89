package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.LinkedList;

public class FinalRobot {
    private LinkedList<Subsystem> subsystems;
    public FinalRobot(HardwareMap hardwareMap) {

    }

    Runnable subsystemEventLoop = new Runnable() {
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    for (Subsystem subsystem : subsystems) {
                        subsystem.update();
                    }
                } catch (Throwable e) {
                    Log.wtf("Robot", "wtf: ", e);
                }
            }
        }
    };

}
