package org.firstinspires.ftc.teamcode;

public class Latch {
    private boolean prev = false;

    public boolean update(boolean value) {
        boolean ret = !prev && value;
        prev = value;
        return ret;
    }
}
