package com.robot.balancingandroid;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Environment;
import android.os.SystemClock;
import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

import static android.os.SystemClock.*;

/**
 * Created by gary on 3/30/15.
 */
public class Estimator
{

    private static final String TAG = "BalancingAndroid";

    public interface EstimatorListener {
        public void onNewState(Mat state);
    }
    private EstimatorListener estimatorListener;

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

    private double thetaOffset;
    private double thetaSum = 0.0;
    private int initCount = 0;
    private final int initNum = 20;

    private double accelLPFCuttoffFrequency = 10.0;
    private double alphaAccelLPF = Math.exp(-accelLPFCuttoffFrequency * 2*Math.PI * sensorSampleTimeS);

    private double rawThetaMes = 0;
    private double thetaMes = 0;
    private boolean newThetaMes = false;

    private double flowMes = 0;
    private boolean newFlowMes = false;
    private double FLOW_PIXELS_TO_METERS = .011/200;

    private boolean updateDone = true;

    //matricies
    public Mat state; //theta, theta_dot, omega (rad, rad/s, rad/s)
    private Mat A;
    private Mat B;
    private Mat C;
    private Mat L;
    private Mat u;

    //constants
    private float g = 9.81f;// m/s2

    private float m1 = 0.1162f;// kg
    private float l1 = .04f;// m

    private float m2 = 0.5647f;// kg
    private float l2 = 0.28f;// m
    private float I2 = 0.0481f;

    private long lastTime;

    private FileOutputStream outputStream;

    public Estimator(SensorManager sm, EstimatorListener el) {

        sensorManager = sm;
        estimatorListener = el;

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
        val = l1;
        C.put(2,2,val);

        L = Mat.zeros(3, 3, CvType.CV_32FC1);

        u = Mat.zeros(1, 1, CvType.CV_32FC1);

        if (sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE) != null)
            gyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        if (sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) != null)
            accel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        String filename = "myFile.txt";

        if( Environment.MEDIA_MOUNTED.equals(Environment.getExternalStorageState()))
        {
            File myDir = new File(Environment.getExternalStoragePublicDirectory(
                    Environment.DIRECTORY_DOWNLOADS), "myFolder");
            //Log.i(TAG, myDir.getPath().toString());
            if (!myDir.mkdirs()) {
                Log.e(TAG, "Directory not created");
            }
            File file = new File(myDir.getPath(), filename);

            if(file.exists()) file.delete();

            try {
                outputStream = new FileOutputStream(file);
                //outputStream.write(test.getBytes());
                //outputStream.close();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        else
            Log.e(TAG, "media bocked");
    }

    public void registerListeners() {
        if (gyro != null)
            sensorManager.registerListener(sensorListener, gyro, sensorSampleTimeUs);

        if (accel != null)
            sensorManager.registerListener(sensorListener, accel, sensorSampleTimeUs);

        lastTime = elapsedRealtimeNanos();
    }

    public void unregisterListeners() {

        try {
            outputStream.close();
        } catch (IOException e) {
            Log.e(TAG, "could not close");
            //e.printStackTrace();
        }
        sensorManager.unregisterListener(sensorListener);
    }

    private SensorEventListener sensorListener = new SensorEventListener()
    {
        @Override
        public void onSensorChanged(SensorEvent event) {
            if (initCount < initNum) {
                if (event.sensor == accel) {
                    thetaSum += Math.atan2(-event.values[2], event.values[1]);
                    initCount++;
                }
            } else if (initCount == initNum) {
                thetaOffset = thetaSum / initNum;
                initCount++;
                Log.i("BalancingAndroid", "theta offset initialized: " + thetaOffset);
            } else {
                if (event.sensor == gyro)
                {
                    rawThetaDotMes = event.values[0];
                    thetaDotMes = alphaGyroLPF*thetaDotMes + (1 - alphaGyroLPF)*rawThetaDotMes;
                    newThetaDotMes = true;
                }
                else if (event.sensor == accel)
                {
                    rawThetaMes = Math.atan2(-event.values[2],event.values[1]) - thetaOffset;
                    thetaMes = alphaAccelLPF*thetaMes + (1 - alphaAccelLPF)*rawThetaMes;
                    newThetaMes = true;

                    StringBuilder sb = new StringBuilder();
                    sb.append(SystemClock.elapsedRealtime()).append(", ").append(thetaMes).append(", ").append(thetaDotMes).append(", ").append(flowMes).append("\n");
                    try {
                        outputStream.write(sb.toString().getBytes());
                    } catch (IOException e) {
                        Log.e(TAG, "could not write");
                        e.printStackTrace();
                    }

                    if(updateDone) {
                        updateEstimate();
                    } else {
                        Log.i("BalancingAndroid", "skipped an update");
                    }
                }
            }

        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {

        }
    };

    public void updateEstimate() {

        updateDone = false;

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
            val = 2.6635f;
            L.put(0,0,val);
            val = 12.1742f;
            L.put(1,0,val);
            val = 0.0510f;
            L.put(2,0,val);
            y.put(0,0,thetaMes);
            newThetaMes = false;
        }
        if(newThetaDotMes)
        {
            val = 1.4172f;
            L.put(0,1,val);
            val = 9.8864f;
            L.put(1,1,val);
            val = -0.3343f;
            L.put(2,1,val);
            y.put(1,0,thetaDotMes);
            newThetaDotMes = false;
        }
        if(newFlowMes)
        {
            val = 0.1985f;
            L.put(0,2,val);
            val = -11.1802f;
            L.put(1,2,val);
            val = 521.0064f;
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

        float[] n = new float[3];
        state.get(0,0,n);
        //Log.i("BalancingAndroid", "theta = " + n[0] + " rad,  thetadot = " + n[1] + " rad/s,  omega = " + n[2] + " rad/s");

        estimatorListener.onNewState(state);

        updateDone = true;
    }

    public void onFlowChanged(double flow) {

//        StringBuilder sb = new StringBuilder();
//        sb.append(SystemClock.elapsedRealtime()).append(", ").append(thetaMes).append(", ").append(thetaDotMes).append(", ").append(flowMes).append("\n");
//        try {
//            outputStream.write(sb.toString().getBytes());
//        } catch (IOException e) {
//            Log.e(TAG, "could not write");
//            e.printStackTrace();
//        }

        if(flow != Double.NaN);
        {
            flowMes = FLOW_PIXELS_TO_METERS * flow;
            newFlowMes = true;
        }
    }

    public Mat getState() {
        return state;
    }

    public void onInputChanged(float input) {
        float val = input;
        u.put(0,0,val);
    }
}
