package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;

public class RevModuleSensorMonitor implements Subsystem {
    private LynxModule revModule;
    private LynxGetBulkInputDataResponse lastResponse;
    private long lastTs;

    public RevModuleSensorMonitor(LynxModule revModule) {
        this.revModule = revModule;

        // prepopulate the cache with a read
        int attempts = 0;
        while (lastResponse == null) {
            lastResponse = getResponse();
            lastTs = System.nanoTime();
            if (attempts++ >= 3) {
                Log.e("RevModuleSensorMonitor", "Initialization failed; is the Rev hub talking?");
                return;
            }
        }

    }
    public void update() {
        LynxGetBulkInputDataResponse response = getResponse();
        if (response == null) return; // nothing useful this time around :/

        lastResponse = response;
        lastTs = System.nanoTime();
    }

    public LynxGetBulkInputDataResponse getLastResponse() {
        return lastResponse;
    }
    public long getLastResponseTimestamp() {
        return lastTs;
    }

    private LynxGetBulkInputDataResponse getResponse() {
        // an object if successful, else null
        // based on code by 8367 ACME Robotics
        try {
            return new LynxGetBulkInputDataCommand(revModule).sendReceive();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (Exception e) {
            Log.e("RevModuleSensorMonitor", "Error in sendRecieve(): ", e);
        }
        return null;
    }
}
