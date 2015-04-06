package com.robot.balancingandroid;

import android.os.SystemClock;
import android.util.Log;

import org.opencv.android.CameraBridgeViewBase;
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

    List<ImageProcessorListener> listeners;

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

    // optical flow stuff
    Mat prevFrame;
    static final int maxFeatures = 300;
    static final double qualityLevel = 0.1;
    static final double minDistance = 2.0;
    boolean initialized;

    static final Rect roi = new Rect(60, 20, 200, 200);

    long prevTime;

    ImageProcessor() {
        listeners = new ArrayList<>();
        prevFrame = new Mat();
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

        long time = SystemClock.elapsedRealtime();
        Mat frame = inputFrame.gray();

        double dt = ((double)(time - prevTime)) / 1e3;
        prevTime = time;

        if (!initialized)
        {
            frame.copyTo(prevFrame);
            initialized = true;
            return frame;
        }

        Mat display = new Mat();
        Imgproc.cvtColor(prevFrame, display, Imgproc.COLOR_GRAY2BGR);
        Core.rectangle(display, new Point(roi.x, roi.y), new Point(roi.x + roi.width, roi.y + roi.height), new Scalar(255,255,255));

        Mat frameRoi = new Mat(frame, roi);
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

            double flow = (flowSum / flowCount) / dt;

            for (ImageProcessorListener listener : listeners) {
                listener.onNewFlow(flow);
            }
        } else {
            Log.e(TAG, "No points found!");
        }

        frame.copyTo(prevFrame);

        return display;
    }
}
