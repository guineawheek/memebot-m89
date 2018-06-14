package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;

public class EncoderLocalizer extends Subsystem {

    private RevModuleSensorMonitor hubMonitor;
    private AngularRateSensor gyro;
    private double unitsCoefficient;
    private double x;
    private double y;

    private double xOffset;
    private double yOffset;

    private double lastX;
    private double lastY;

    private int encXIndex;
    private int encYIndex;

    public EncoderLocalizer(RevModuleSensorMonitor hubMonitor, int encXIndex, int encYIndex, double xOffset, double yOffset, AngularRateSensor gyro, double unitsCoefficient) {
        this.hubMonitor = hubMonitor;
        this.gyro = gyro;
        this.encXIndex = encXIndex;
        this.encYIndex = encYIndex;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
    }

    public double getDeltaTheta() {
        return 0; // stub for now, return in radians
    }

    public synchronized void update() {
        LynxGetBulkInputDataResponse data = hubMonitor.getLastResponse();
        double deltaTheta = getDeltaTheta();
        double deltaX = data.getEncoder(encXIndex) - lastX - xOffset * deltaTheta;
        double deltaY = data.getEncoder(encYIndex) - lastY - yOffset * deltaTheta;

        // it's just an array access so this is extremely cheap; see RevModuleSensorMonitor#update
        lastX = data.getEncoder(encXIndex);
        lastY = data.getEncoder(encYIndex);

        x += deltaX * Math.cos(deltaTheta) - deltaY * Math.sin(deltaTheta);
        y += deltaX * Math.sin(deltaTheta) + deltaY * Math.cos(deltaTheta);
    }

}
