package com.robot.balancingandroid;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

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

        K = Mat.zeros(3, 3, CvType.CV_32FC1);
        float val = 1;
        K.put(0, 0, val);
        //todo initialize k correctly...
    }

    public void calculateCommand(Mat state) {
        Mat empty = Mat.zeros(3,1, CvType.CV_32FC1);
        Mat u = Mat.zeros(1,1, CvType.CV_32FC1);
        Core.gemm(K, state, 1, empty, 0, u, 0);//todo there should be a negative in front of K

        float[] val = new float[1];
        u.get(0,0,val);
        controllerListener.onNewCommand(val[1]);
    }
}