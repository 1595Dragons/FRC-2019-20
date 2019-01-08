package frc.robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;

public class Vision {

    public void findTape() {
        CvSink cvSink = CameraServer.getInstance().getVideo();
        CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);
        Mat mat = new Mat();

        if (cvSink.grabFrame(mat) == 0) {
            outputStream.notifyError(cvSink.getError());
        }

        Mat hsv = new Mat();
        Mat thres = new Mat();
        Imgproc.cvtColor(mat, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, new Scalar(0, 0, 240), new Scalar(255, 255, 255), thres);
        Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
        // outputStream.putFrame(mat);
        outputStream.putFrame(thres);

        thres.release();
        hsv.release();
        mat.release();

    }

}