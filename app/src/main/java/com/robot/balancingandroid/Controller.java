package com.robot.balancingandroid;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

/**
 * Created by gary on 4/3/15.
 */
public class Controller {

    public interface ControllerListener {
        public void onNewCommand(double omega);
    }

    private ControllerListener controllerListener;
    private Mat K;

    public Controller(ControllerListener contlist) {
        controllerListener = contlist;

        K = Mat.zeros(1, 3, CvType.CV_32FC1);
//        float val = -158.5f;
//        K.put(0, 0, val);
//        val = -31.45f;
//        K.put(0, 1, val);
//        val = -0.22282f;
//        K.put(0, 2, val);

        setGains(-140.0f, -120.0f, -0.2282f);
    }

    public void setGains(float k1, float k2, float k3) {
        K.put(0, 0, k1);
        K.put(0, 1, k2);
        K.put(0, 2, k3);
    }

    public float[] getGains() {
        float gains[] = new float[3];
        K.get(0,0,gains);
        return gains;
    }

    public void calculateCommand(Mat state) {
        Mat empty = Mat.zeros(3,1, CvType.CV_32FC1);
        Mat u = Mat.zeros(1,1, CvType.CV_32FC1);
        Core.gemm(K, state, 1, empty, 0, u, 0);
        Scalar neg = new Scalar(-1);
        Core.multiply(u, neg, u);

        float[] val = new float[1];
        u.get(0,0,val);
        controllerListener.onNewCommand(val[0]);
    }
}
