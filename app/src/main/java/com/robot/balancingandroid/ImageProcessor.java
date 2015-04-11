package com.robot.balancingandroid;

import android.graphics.Bitmap;
import android.os.AsyncTask;
import android.os.SystemClock;
import android.util.Log;
import android.widget.ImageView;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.Utils;
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

import java.util.ArrayList;
import java.util.List;

/**
 * Image processing class
 */
public class ImageProcessor implements CameraBridgeViewBase.CvCameraViewListener2 {

    public interface ImageProcessorListener {
        public void onNewFlow(double flow);
    }

    private List<ImageProcessorListener> listeners;

    public void registerListener(ImageProcessorListener listener)
    {
        if (listener != null) {
            listeners.add(listener);
        }
    }

    public void unregisterListener(ImageProcessorListener listener)
    {
        if (listener != null) {
            listeners.remove(listener);
        }
    }

    private static final String TAG = "ImageProcessor";

    private ImageView flowView;

    private class OpticalFlowTask extends AsyncTask<Mat, Integer, Double> {

        private Mat prevFrame;
        private double dt;
        private Mat display;

        private static final int maxFeatures = 300;
        private static final double qualityLevel = 0.1;
        private static final double minDistance = 2.0;

        public OpticalFlowTask() {
            super();
            prevFrame = new Mat();
        }

        public void setPrevFrame(Mat frame) {
            frame.copyTo(prevFrame);
        }

        public void setTime(double dt) {
            this.dt = dt;
        }

        @Override
        protected Double doInBackground(Mat... images) {

            display = new Mat();
            Imgproc.cvtColor(prevFrame, display, Imgproc.COLOR_GRAY2BGR);
            Core.rectangle(display, new Point(roi.x, roi.y), new Point(roi.x + roi.width, roi.y + roi.height), new Scalar(255,255,255));

            Mat frameRoi = new Mat(images[0], roi);
            Mat prevFrameRoi = new Mat(prevFrame, roi);
            Mat displayRoi = new Mat(display, roi);

            MatOfPoint features = new MatOfPoint();
            Imgproc.goodFeaturesToTrack(prevFrameRoi, features, maxFeatures, qualityLevel, minDistance);
            Point featuresArray[] = features.toArray();

            if (featuresArray.length > 0) {

                MatOfPoint2f prevPts = new MatOfPoint2f();
                prevPts.fromArray(featuresArray);

                MatOfPoint2f nextPts = new MatOfPoint2f();
                MatOfByte status = new MatOfByte();
                MatOfFloat err = new MatOfFloat();

                Video.calcOpticalFlowPyrLK(prevFrameRoi, frameRoi, prevPts, nextPts, status, err);

                Point prevPtsArray[] = prevPts.toArray();
                Point nextPtsArray[] = nextPts.toArray();
                byte statusArray[] = status.toArray();

                int flowCount = 0;
                double flowSum = 0.0;
                for (int i = 0; i < prevPtsArray.length; i++) {
                    if (statusArray[i] != 0) {
                        Core.circle(displayRoi, prevPtsArray[i], 5, new Scalar(0, 255, 0));
                        Core.line(displayRoi, prevPtsArray[i], nextPtsArray[i], new Scalar(0, 0, 255));

                        flowCount++;
                        flowSum += nextPtsArray[i].x - prevPtsArray[i].x;

                    } else {
                        Core.circle(displayRoi, prevPtsArray[i], 5, new Scalar(255, 0, 0));
                    }
                }

                return (flowSum / flowCount) / dt;

            } else {
                Log.e(TAG, "No points found!");
                return null;
            }
        }

        @Override
        protected void onPostExecute(Double result) {
            if (result != null) {
                for (ImageProcessorListener listener : listeners) {
                    listener.onNewFlow(result);
                }
            }

            if (flowView != null && display != null) {
                Bitmap bm = Bitmap.createBitmap(display.cols(), display.rows(), Bitmap.Config.ARGB_8888);
                Utils.matToBitmap(display, bm);
                flowView.setImageBitmap(bm);
            }
        }
    }

    // optical flow stuff
    private Mat prevFlowFrame;
    private long prevFlowTime;
    private OpticalFlowTask opticalFlowTask;
    private boolean initialized;

    private Rect roi = new Rect(60, 20, 200, 200);

    ImageProcessor() {
        listeners = new ArrayList<>();
        prevFlowFrame = new Mat();
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        initialized = false;
    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        if (!initialized) {
            inputFrame.gray().copyTo(prevFlowFrame);
            initialized = true;
            return inputFrame.rgba();
        }

        if (opticalFlowTask == null || opticalFlowTask.getStatus() == OpticalFlowTask.Status.FINISHED) {

            long flowTime = SystemClock.elapsedRealtime();
            double dt = ((double)(flowTime - prevFlowTime)) / 1e3;
            prevFlowTime = flowTime;

            opticalFlowTask = new OpticalFlowTask();
            opticalFlowTask.setPrevFrame(prevFlowFrame);
            opticalFlowTask.setTime(dt);
            opticalFlowTask.execute(inputFrame.gray());

            inputFrame.gray().copyTo(prevFlowFrame);
        }

        Mat display = inputFrame.rgba();
        Core.rectangle(display, new Point(roi.x, roi.y), new Point(roi.x + roi.width, roi.y + roi.height), new Scalar(255,255,255));
        return display;
    }

    public void setRoi(Rect roi) {
        if (roi != null) {
            this.roi = roi;
            initialized = false;
        }
    }

    public void setFlowView(ImageView view) {
        flowView = view;
    }
}
