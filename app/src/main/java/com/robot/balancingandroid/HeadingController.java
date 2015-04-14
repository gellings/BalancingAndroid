package com.robot.balancingandroid;

import android.os.SystemClock;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by dpkoch on 4/13/15.
 */
public class HeadingController {

    public interface HeadingControllerListener {
        public void onNewHeadingCommand(int command);
    }

    public void registerListener(HeadingControllerListener listener) {
        if (listener != null) {
            listeners.add(listener);
        }
    }

    public void unregisterListener(HeadingControllerListener listener) {
        if (listener != null) {
            listeners.remove(listener);
        }
    }

    private List<HeadingControllerListener> listeners;

    private double kp;
    private double ki;

    private double integrator;
    private long prevTime;
    private boolean initialized;

    public HeadingController() {
        listeners = new ArrayList<>();
        integrator = 0;

        kp = 0.1;
        ki = 0.00;

        initialized = false;
    }

    public void calculateCommand(double angle) {

        long time = SystemClock.elapsedRealtime();

        if (!initialized) {
            integrator = 0.0;
            prevTime = time;
            initialized = true;
            return;
        }

        double dt = ((double)(time - prevTime)) / 1e3;
        integrator += angle * dt;

        int command = (int)(kp*angle + ki*integrator);
        for (HeadingControllerListener listener : listeners) {
            listener.onNewHeadingCommand(command);
        }
    }
}
