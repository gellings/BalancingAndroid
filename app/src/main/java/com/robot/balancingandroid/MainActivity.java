package com.robot.balancingandroid;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.core.Mat;

import android.app.Activity;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.hardware.SensorManager;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;

public class MainActivity extends Activity implements CvCameraViewListener2 {

    private static final String TAG = "BalancingAndroid";

    private CameraBridgeViewBase openCvCameraView;

    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status)
            {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    openCvCameraView.enableView();
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    PulseGenerator noise;
    Thread noiseThread;

    Estimator estimator;
    Thread estimatorThread;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        estimator = new Estimator((SensorManager) this.getSystemService(Context.SENSOR_SERVICE));

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
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    @Override
    protected void onResume()
    {
        super.onResume();
        //estimator.registerListeners();

        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_10, this, loaderCallback);
    }

    @Override
    protected void onPause()
    {
        //estimator.unregisterListeners();
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

        if(!noise.isPlaying())
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
        return inputFrame.rgba();
    }
}
