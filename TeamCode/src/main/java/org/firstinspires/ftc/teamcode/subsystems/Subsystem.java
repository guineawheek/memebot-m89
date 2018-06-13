package org.firstinspires.ftc.teamcode.subsystems;

public abstract class Subsystem {
    private boolean busy = false;
    public abstract void update();
    public synchronized void setBusy() {
        busy = true;
    }
    public synchronized boolean isBusy() {
        return busy;
    }
    public synchronized void setDone() {
        busy = false;
    }

}
