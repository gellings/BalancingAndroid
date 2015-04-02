package com.robot.balancingandroid;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;

import android.app.Activity;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.hardware.SensorManager;
import android.os.Environment;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;

import java.io.File;
import java.io.FileOutputStream;

public class MainActivity extends Activity implements CvCameraViewListener2 {

    private static final String TAG = "BalancingAndroid";

    static {
        if (OpenCVLoader.initDebug()) {
            Log.i(TAG, "OpenCV successfully loaded");
        } else {
            Log.e(TAG, "OpenCV failed to load");
        }
    }

    private CameraBridgeViewBase openCvCameraView;

    // optical flow stuff
    Mat prevFrame;
    static final int maxFeatures = 500;
    static final double qualityLevel = 0.1;
    static final double minDistance = 4.0;
    boolean initialized;

    static final Rect roi = new Rect(100, 100, 400, 400);

    PulseGenerator noise;
    Thread noiseThread;

    Estimator estimator;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        prevFrame = new Mat();

        estimator = new Estimator(this, (SensorManager) this.getSystemService(Context.SENSOR_SERVICE));

        noise = new PulseGenerator();
        noiseThread = new Thread(noise);

        openCvCameraView = (CameraBridgeViewBase) findViewById(R.id.java_surface_view);
        openCvCameraView.setCvCameraViewListener(this);
    }


    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    protected void onResume()
    {
        super.onResume();
        estimator.registerListeners();

        openCvCameraView.enableView();
        initialized = false;
    }

    @Override
    protected void onPause()
    {
        estimator.unregisterListeners();

        super.onPause();
        if (openCvCameraView != null)
        {
            openCvCameraView.disableView();
        }
    }

    @Override
    protected void onStart()
    {
        if (!noiseThread.isAlive())
            noiseThread.start();

        noise.setPulsePercent(50, 0);
        noise.setPulsePercent(50, 2);

        if(noise.isPlaying()) //change this to make the noise work
            noise.togglePlayback();

        super.onStart();
    }

    @Override
    protected void onStop()
    {
        noise.stop();
        super.onStop();
    }

    @Override
    protected void onDestroy()
    {
        noise.stop();
        // soundToggleButton.setChecked(false);
        super.onDestroy();
        if (openCvCameraView != null)
        {
            openCvCameraView.disableView();
        }
    }

    public void onCameraViewStarted(int width, int height)
    {
    }

    public void onCameraViewStopped()
    {
    }

    public Mat onCameraFrame(CvCameraViewFrame inputFrame)
    {
        Mat frame = inputFrame.gray();

        if (!initialized)
        {
            frame.copyTo(prevFrame);
            initialized = true;
        }

        Mat display = new Mat();
        Imgproc.cvtColor(prevFrame, display, Imgproc.COLOR_GRAY2BGR);

        MatOfPoint features = new MatOfPoint();
        Imgproc.goodFeaturesToTrack(prevFrame, features, maxFeatures, qualityLevel, minDistance);
        Point featuresArray[] = features.toArray();

        if (featuresArray.length > 0) {

            MatOfPoint2f prevPts = new MatOfPoint2f();
            prevPts.fromArray(featuresArray);

            MatOfPoint2f nextPts = new MatOfPoint2f();
            MatOfByte status = new MatOfByte();
            MatOfFloat err = new MatOfFloat();

            Video.calcOpticalFlowPyrLK(prevFrame, frame, prevPts, nextPts, status, err);

            Point prevPtsArray[] = prevPts.toArray();
            Point nextPtsArray[] = nextPts.toArray();
            byte statusArray[] = status.toArray();

            for (int i = 0; i < prevPtsArray.length; i++) {
                if (statusArray[i] != 0) {
                    Core.circle(display, prevPtsArray[i], 5, new Scalar(0, 255, 0));
                    Core.line(display, prevPtsArray[i], nextPtsArray[i], new Scalar(0, 0, 255));
                } else {
                    Core.circle(display, prevPtsArray[i], 5, new Scalar(255, 0, 0));
                }
            }
        } else {
            Log.e(TAG, "No points found!");
        }

        frame.copyTo(prevFrame);
        return display;
    }
}
