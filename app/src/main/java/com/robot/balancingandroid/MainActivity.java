package com.robot.balancingandroid;

import android.app.Activity;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.OpenCVLoader;

public class MainActivity extends Activity {

    private static final String TAG = "BalancingAndroid";

    static {
        if (OpenCVLoader.initDebug()) {
            Log.i(TAG, "OpenCV successfully loaded");
        } else {
            Log.e(TAG, "OpenCV failed to load");
        }
    }

    private CameraBridgeViewBase openCvCameraView;
    private ImageProcessor imageProcessor;

    PulseGenerator noise;
    Thread noiseThread;

    Estimator estimator;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        imageProcessor = new ImageProcessor();

        estimator = new Estimator((SensorManager) this.getSystemService(Context.SENSOR_SERVICE));

        noise = new PulseGenerator();
        noiseThread = new Thread(noise);

        openCvCameraView = (CameraBridgeViewBase) findViewById(R.id.java_surface_view);
        openCvCameraView.setCvCameraViewListener(imageProcessor);
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
}
