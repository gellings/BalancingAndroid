package com.robot.balancingandroid;

import android.app.Activity;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;

import org.opencv.android.JavaCameraView;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
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

    private int imageWidth;
    private int imageHeight;
    private Rect roi;

    private EditText roiX;
    private EditText roiY;
    private EditText roiWidth;
    private EditText roiHeight;

    private PulseGenerator noise;
    private Thread noiseThread;

    private Estimator estimator;
    private Controller controller;
    private HeadingController headingController;

    private double omega_dotCurrent = 0;

    private int headingCommand = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        estimator = new Estimator((SensorManager) this.getSystemService(Context.SENSOR_SERVICE), estimatorListener);
        controller = new Controller(controllerListener);

        headingController = new HeadingController();
        headingController.registerListener(headingControllerListener);

        noise = new PulseGenerator(pulseGenListener);
        noiseThread = new Thread(noise);

        imageWidth = 320;
        imageHeight = 240;
        roi = new Rect(0, 60, 150, 180);

        roiX = (EditText)findViewById(R.id.roi_x);
        roiY = (EditText)findViewById(R.id.roi_y);
        roiWidth = (EditText)findViewById(R.id.roi_width);
        roiHeight = (EditText)findViewById(R.id.roi_height);

        roiX.setText(Integer.toString(roi.x));
        roiY.setText(Integer.toString(roi.y));
        roiWidth.setText(Integer.toString(roi.width));
        roiHeight.setText(Integer.toString(roi.height));

        imageProcessor = new ImageProcessor();
        imageProcessor.setRoi(roi);
        imageProcessor.setFlowView((ImageView) findViewById(R.id.flow_view));
        imageProcessor.setLinesView((ImageView) findViewById(R.id.lines_view));
        imageProcessor.registerListener(imageProcessorListener);

        cameraView = (JavaCameraView) findViewById(R.id.main_image);
        cameraView.setCameraIndex(JavaCameraView.CAMERA_ID_FRONT);
        cameraView.SetCaptureFormat(Highgui.CV_CAP_ANDROID_GREY_FRAME);
        cameraView.setMaxFrameSize(imageWidth, imageHeight);
        cameraView.setCvCameraViewListener(imageProcessor);

        // initialize values
        Button updateRoiButton = (Button) findViewById(R.id.update_roi);
        updateRoiButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                try {

                    int x = Integer.parseInt(roiX.getText().toString());
                    int y = Integer.parseInt(roiY.getText().toString());
                    int width = Integer.parseInt(roiWidth.getText().toString());
                    int height = Integer.parseInt(roiHeight.getText().toString());

                    if (x <= imageWidth && x + width <= imageWidth
                            && y <= imageHeight && y + height <= imageHeight) {
                        roi = new Rect(x, y, width, height);
                        imageProcessor.setRoi(roi);
                    }
                    else {
                        roiX.setText(Integer.toString(roi.x));
                        roiY.setText(Integer.toString(roi.y));
                        roiWidth.setText(Integer.toString(roi.width));
                        roiHeight.setText(Integer.toString(roi.height));
                    }
                } catch (Exception e) {
                    roiX.setText(Integer.toString(roi.x));
                    roiY.setText(Integer.toString(roi.y));
                    roiWidth.setText(Integer.toString(roi.width));
                    roiHeight.setText(Integer.toString(roi.height));
                }
            }
        });
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
        if (cameraView != null)
        {
            cameraView.disableView();
        }
    }

    private ImageProcessor.ImageProcessorListener imageProcessorListener = new ImageProcessor.ImageProcessorListener() {
        @Override
        public void onNewFlow(double flow) {
            if (estimator != null) {
                //Log.i(TAG, "Flow = " + flow + " pixels/s");
                estimator.onFlowChanged(flow);
            }
        }

        @Override
        public void onNewAngle(double angle) {
            headingController.calculateCommand(angle);
            Log.i(TAG, "Angle = " + angle);
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
            double deltaT = ((double)(time - lastTime)) / 1e9;
            lastTime = time;
            omega += omega_dot*deltaT;

            //saturate
            if(omega > OMEGA_MAX)
                omega = OMEGA_MAX;
            if(omega < -OMEGA_MAX)
                omega = -OMEGA_MAX;

            int command0 = 50 - ((int)( 50 * omega/OMEGA_MAX) + headingCommand);
            int command2 = 43 + ((int)( 50 * omega/OMEGA_MAX) - headingCommand);

            if (command0 < 0) { command0 = 0; }
            if (command0 > 100) { command0 = 100; }

            if (command2 < 0) { command2 = 0; }
            if (command2 > 100) { command2 = 100; }

            noise.setPulsePercent(command0, 0);
            noise.setPulsePercent(command2, 2);
        }
    };

    private HeadingController.HeadingControllerListener headingControllerListener
            = new HeadingController.HeadingControllerListener() {
        @Override
        public void onNewHeadingCommand(int command) {
            headingCommand = command;
            Log.i(TAG, "Heading command: " + command);
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
