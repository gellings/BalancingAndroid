package com.robot.balancingandroid;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

/**
 * Created by gary on 3/30/15.
 */
public class Estimator implements Runnable
{
    private SensorManager sensorManager;

    private Sensor gyro = null;
    private Sensor accel = null;

    private int sensorSampleTimeUs = 10000;
    private double sensorSampleTimeS = sensorSampleTimeUs / 1.0e6;

    private double gyroLPFCuttoffFrequency = 50.0;
    private double alphaGyroLPF = Math.exp(-gyroLPFCuttoffFrequency * 2*Math.PI * sensorSampleTimeS);

    private double rawThetaDotMes = 0;
    private double thetaDotMes = 0;

    private double accelLPFCuttoffFrequency = 10.0;
    private double alphaAccelLPF = Math.exp(-accelLPFCuttoffFrequency * 2*Math.PI * sensorSampleTimeS);

    private double rawThetaMes = 0;
    private double thetaMes = 0;

    public Estimator(SensorManager sm) {

        sensorManager = sm;

        if (sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE) != null)
            gyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        if (sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) != null)
            accel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    }

    public void registerListeners() {
        if (gyro != null)
            sensorManager.registerListener(sensorListener, gyro, sensorSampleTimeUs);

        if (accel != null)
            sensorManager.registerListener(sensorListener, accel, sensorSampleTimeUs);
    }

    public void unregisterListeners() {
        sensorManager.unregisterListener(sensorListener);
    }

    private SensorEventListener sensorListener = new SensorEventListener()
    {
        @Override
        public void onSensorChanged(SensorEvent event) {
            if (event.sensor == gyro)
            {
                rawThetaDotMes = event.values[0];
                thetaDotMes = alphaGyroLPF*thetaDotMes + (1 - alphaGyroLPF)*rawThetaDotMes;
            }
            else if (event.sensor == accel)
            {
                rawThetaMes = Math.atan2(-event.values[2],event.values[1]);
                thetaMes = alphaAccelLPF*thetaMes + (1 - alphaAccelLPF)*rawThetaMes;
            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {

        }
    };

    @Override
    public void run() {
        while(true)
        {
            //estimate...
        }
    }
}
