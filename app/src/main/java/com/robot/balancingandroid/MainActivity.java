package com.robot.balancingandroid;

import android.app.Activity;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;

import org.opencv.android.JavaCameraView;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;

public class MainActivity extends Activity {

    private static final String TAG = "BalancingAndroid";

    static {
        if (OpenCVLoader.initDebug()) {
            Log.i(TAG, "OpenCV successfully loaded");
        } else {
            Log.e(TAG, "OpenCV failed to load");
        }
    }

    private JavaCameraView cameraView;
    private ImageProcessor imageProcessor;

    private PulseGenerator noise;
    private Thread noiseThread;

    private Estimator estimator;
    private Controller controller;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        imageProcessor = new ImageProcessor();
        imageProcessor.registerListener(imageProcessorListener);

        estimator = new Estimator((SensorManager) this.getSystemService(Context.SENSOR_SERVICE), estimatorListener);
        controller = new Controller(controllerListener);

        noise = new PulseGenerator();
        noiseThread = new Thread(noise);

        cameraView = (JavaCameraView) findViewById(R.id.java_surface_view);
        cameraView.setCameraIndex(JavaCameraView.CAMERA_ID_FRONT);
        cameraView.SetCaptureFormat(Highgui.CV_CAP_ANDROID_GREY_FRAME);
        cameraView.setMaxFrameSize(320, 240);
        cameraView.setCvCameraViewListener(imageProcessor);
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

        cameraView.enableView();
    }

    @Override
    protected void onPause()
    {
        estimator.unregisterListeners();

        super.onPause();
        if (cameraView != null)
        {
            cameraView.disableView();
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
        if (cameraView != null)
        {
            cameraView.disableView();
        }
    }

    ImageProcessor.ImageProcessorListener imageProcessorListener = new ImageProcessor.ImageProcessorListener() {
        @Override
        public void onNewFlow(double flow) {
            if (estimator != null) {
//                estimator.onFlowChanged(flow);
            }
        }
    };

    private Controller.ControllerListener controllerListener = new Controller.ControllerListener() {

        @Override
        public void onNewCommand(double omega) {
//            noise.setPulsePercent();
        }
    };

    private Estimator.EstimatorListener estimatorListener = new Estimator.EstimatorListener() {

        @Override
        public void onNewState(Mat state) {
//            controller.calculateCommand(state);
        }
    };
}
