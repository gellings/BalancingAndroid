package com.robot.balancingandroid;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import static android.os.SystemClock.*;

/**
 * Created by gary on 3/30/15.
 */
public class Estimator
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
    private boolean newThetaDotMes = false;

    private double accelLPFCuttoffFrequency = 10.0;
    private double alphaAccelLPF = Math.exp(-accelLPFCuttoffFrequency * 2*Math.PI * sensorSampleTimeS);

    private double rawThetaMes = 0;
    private double thetaMes = 0;
    private boolean newThetaMes = false;

    private double flowMes = 0;
    private boolean newFlowMes = false;

    //matricies
    public Mat state; //theta, theta_dot, omega (rad, rad/s, rad/s)
    private Mat A;
    private Mat B;
    private Mat C;
    private Mat L;
    private Mat u;

    //constants
    private float g = 9.81f;// m/s2

    private float m1 = 2f;// kg
    private float l1 = 1.3f;// m

    private float m2 = 2f;// kg
    private float l2 = 1.3f;// m
    private float I2 = 7f;

    private long lastTime;

    public Estimator(SensorManager sm) {

        sensorManager = sm;

        state = Mat.zeros(3, 1, CvType.CV_32FC1);

        A = Mat.zeros(3, 3, CvType.CV_32FC1);
        float val = 1;
        A.put(0,1,val);
        val = g*l2*m2 / (l1*l1*m1 + 2*l1*l2*m1 + l2*l2*m1 + l1*l1*m2 + I2);
        A.put(1,0,val);

        B = Mat.zeros(3, 1, CvType.CV_32FC1);
        val = (-l1*l1*m1 - l1*l2*m1 - l1*l1*m2) / (l1*l1*m1 + 2*l1*l2*m1 + l2*l2*m1 + l1*l1*m2 + I2);
        B.put(1,0,val);
        val = 1;
        B.put(2,0,val);

        C = Mat.eye(3,3,CvType.CV_32FC1);
        val = l2;
        C.put(2,2,val);

        L = Mat.zeros(3, 3, CvType.CV_32FC1);

        u = Mat.zeros(1, 1, CvType.CV_32FC1);

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

        lastTime = elapsedRealtimeNanos();
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
                newThetaDotMes = true;
            }
            else if (event.sensor == accel)
            {
                rawThetaMes = Math.atan2(-event.values[2],event.values[1]);
                thetaMes = alphaAccelLPF*thetaMes + (1 - alphaAccelLPF)*rawThetaMes;
                newThetaMes = true;

                updateEstimate();
            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {

        }
    };

    public void updateEstimate() {

        Mat state_dot = Mat.zeros(3,1, CvType.CV_32FC1);
        Mat firstTerm = Mat.zeros(3,1, CvType.CV_32FC1);
        Mat secondTerm = Mat.zeros(3,1, CvType.CV_32FC1);
        Mat thirdTerm = Mat.zeros(3,1, CvType.CV_32FC1);
        Mat y = Mat.zeros(3,1, CvType.CV_32FC1);
        Mat empty = Mat.zeros(3,1, CvType.CV_32FC1);

        Core.gemm(A, state, 1, empty, 0, firstTerm, 0);
        Core.gemm(B, u, 1, empty, 0, secondTerm, 0);

        float val;
        if(newThetaMes)
        {
            val = .1f; //todo
            L.put(0,0,val);
            y.put(0,0,thetaMes);
            newThetaMes = false;
        }
        if(newThetaDotMes)
        {
            val = .1f;
            L.put(1,1,val);
            y.put(1,0,thetaDotMes);
            newThetaDotMes = false;
        }
        if(newFlowMes)
        {
            val = .1f;
            L.put(2,2,val);
            y.put(2,0,flowMes);
            newFlowMes = false;
        }

        Core.gemm(C, state, 1, empty, 0, thirdTerm, 0);
        Core.subtract(y, thirdTerm, thirdTerm);
        Core.gemm(L, thirdTerm, 1, empty, 0, thirdTerm, 0);

        Core.add(firstTerm, secondTerm, state_dot);
        Core.add(state_dot,thirdTerm,state_dot);

        long time = elapsedRealtimeNanos();
        double deltaT = (time - lastTime) / 1e9;
        lastTime = time;
        //Scalar dT = new Scalar(deltaT);
        Mat dT = new Mat(1,1,CvType.CV_32FC1);
        dT.put(0,0,deltaT);

        Mat deltaState = Mat.zeros(3,1,CvType.CV_32FC1);
        Core.gemm(state_dot, dT, 1, empty, 0, deltaState, 0);
        Core.add(state, deltaState, state);

        L = Mat.zeros(3, 3, CvType.CV_32FC1);
    }

    public void onFlowChanged(double flow) {
        flowMes = flow;
        newFlowMes = true;
    }

    public Mat getState() {
        return state;
    }

    public void onInputChanged(Mat input) {
        u = input;
    }
}
