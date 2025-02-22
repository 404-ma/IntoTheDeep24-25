 package org.firstinspires.ftc.teamcode.Helper.OpenCV;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


public class OpenCVFun {
    private OpenCvCamera camera;
    private VisionPipeline pipeline;

    public OpenCVFun(@NonNull HardwareMap hwdMap, boolean withLivePreview) {
        pipeline = new VisionPipeline();
        WebcamName webcamName = hwdMap.get(WebcamName.class, "Webcam back");


        if (withLivePreview) {
            int cameraMonitorViewId =hwdMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId" ,"id",hwdMap.appContext.getPackageName()
            );
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        }

        camera.setPipeline(pipeline);

    }


    public void openCameraAndStart() {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

    }

    public void stopCamera() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    // Method to get the values of width, height, distance, and offset

    //public boolean targetFound () { return pipeline.targetDetected;}
    //public double targetDistance(){ return pipeline.targetDistance;}
    //public double targetOffset() { return pipeline.targetOffset;}


    private static class VisionPipeline extends OpenCvPipeline {
        private final Mat hsvMat = new Mat();
        private final Mat thresholdMat = new Mat();
        private final Scalar lowerThreshold = new Scalar(0, 0, 100);
        private final Scalar upperThreshold = new Scalar(100, 100, 255);

        private boolean targetDetected = false;
        private double width = 0;
        private double height = 0;
        private double distance = 0;
        private double offset = 0;


        @Override
        public Mat processFrame(Mat input) {
            Mat brightenedInput = new Mat();
            input.convertTo(brightenedInput, -1, 1, 30);

            Imgproc.cvtColor(brightenedInput, hsvMat, Imgproc.COLOR_RGB2HSV);

            Scalar lowerThreshold1 = new Scalar(0, 100, 150);
            Scalar upperThreshold1 = new Scalar(10, 255, 255);
            Scalar lowerThreshold2 = new Scalar(170, 100, 150);
            Scalar upperThreshold2 = new Scalar(180, 255, 255);

            Core.inRange(hsvMat, lowerThreshold1, upperThreshold1, thresholdMat);
            Mat tempMat = new Mat();
            Core.inRange(hsvMat, lowerThreshold2, upperThreshold2, tempMat);
            Core.bitwise_or(thresholdMat, tempMat, thresholdMat);

            Imgproc.erode(thresholdMat, thresholdMat, new Mat());
            Imgproc.dilate(thresholdMat, thresholdMat, new Mat());

            java.util.List<org.opencv.core.MatOfPoint> contours = new java.util.ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(thresholdMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double maxArea = 0;
            org.opencv.core.Rect largestRect = null;

            for (int i = 0; i < contours.size(); i++) {
                double area = Imgproc.contourArea(contours.get(i));

                if (area > 500) {
                    org.opencv.core.Rect boundingRect = Imgproc.boundingRect(contours.get(i));
                    if (area > maxArea) {
                        maxArea = area;
                        largestRect = boundingRect;
                    }
                }
            }

            targetDetected = (largestRect != null);
            if (targetDetected)  {
                Imgproc.rectangle(input, largestRect.tl(), largestRect.br(), new Scalar(0, 255, 0), 2);

                double knownWidthPx = 112.0;
                double knownDistanceInches = 12.0;

                double observedWidthPx = largestRect.width;
                distance = (knownWidthPx * knownDistanceInches) / observedWidthPx;

                int centerX = input.width() / 2;
                Imgproc.line(input, new org.opencv.core.Point(centerX, 0), new org.opencv.core.Point(centerX, input.height()), new Scalar(255, 0, 0), 2);

                int blockCenterX = largestRect.x + largestRect.width / 2;
                int offsetPx = blockCenterX - centerX;

                double knownWidthAtDistance = knownWidthPx * knownDistanceInches / observedWidthPx;
                offset = (offsetPx * knownWidthAtDistance) / input.width();

                width = largestRect.width;
                height = largestRect.height;

                String text = "Width: " + largestRect.width + " Height: " + largestRect.height + " Distance: " + String.format("%.2f", distance) + " in";
                String offsetText = "Offset: " + String.format("%.2f", offset) + " in";

                Imgproc.putText(input, text, new org.opencv.core.Point(largestRect.x, largestRect.y - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                Imgproc.putText(input, offsetText, new org.opencv.core.Point(largestRect.x, largestRect.y - 30),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            }

            brightenedInput.release();
            tempMat.release();
            hierarchy.release();

            return input;
        }

        public double[] getProcessedData() {
            return new double[]{width, height-2.3, distance, offset};
        }
    }
}
