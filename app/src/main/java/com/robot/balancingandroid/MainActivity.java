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

import static android.os.SystemClock.elapsedRealtimeNanos;

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

    private double omega_dotCurrent = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        imageProcessor = new ImageProcessor();
        imageProcessor.registerListener(imageProcessorListener);

        estimator = new Estimator((SensorManager) this.getSystemService(Context.SENSOR_SERVICE), estimatorListener);
        controller = new Controller(controllerListener);

        noise = new PulseGenerator(pulseGenListener);
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
                Log.d(TAG, "Flow = " + flow + " pixels/s");
                estimator.onFlowChanged(flow);
            }
        }
    };

    private Controller.ControllerListener controllerListener = new Controller.ControllerListener() {

        private long lastTime = elapsedRealtimeNanos();
        private double OMEGA_MAX = 2*Math.PI; //rad/s
        private double omega = 0;

        @Override
        public void onNewCommand(double omega_dot) {

            omega_dotCurrent = omega_dot;

            //integrate
            long time = elapsedRealtimeNanos();
            double deltaT = (time - lastTime) / 1e9;
            lastTime = time;
            omega += omega_dot*deltaT;

            //saturate
            if(omega > OMEGA_MAX)
                omega = OMEGA_MAX;
            if(omega < -OMEGA_MAX)
                omega = -OMEGA_MAX;

            noise.setPulsePercent(0, (int)(50 + 50 * omega/OMEGA_MAX));
            noise.setPulsePercent(2, (int)(50 + 50 * omega/OMEGA_MAX));
        }
    };

    private Estimator.EstimatorListener estimatorListener = new Estimator.EstimatorListener() {

        @Override
        public void onNewState(Mat state) {
            controller.calculateCommand(state);
        }
    };

    private PulseGenerator.PulseGenListener pulseGenListener = new PulseGenerator.PulseGenListener() {

        @Override
        public void onCommandUsed() {
            // send the latest omega to the estimator
            estimator.onInputChanged((float)omega_dotCurrent);
        }
    };
}
