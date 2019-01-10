package frc.robot;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;

public class Vision {

    private GRIPVision grip = new GRIPVision();

    /**
     * Only run this once!
     */
    public void generateTargetImage(VideoSource camera) {
        new Thread(() -> {

            CvSink cvSink = CameraServer.getInstance().getVideo(camera);
            CvSource outputStream = CameraServer.getInstance().putVideo("GRIP", camera.getVideoMode().width,
                    camera.getVideoMode().height);

            Mat source = new Mat();

            while (!Thread.interrupted()) {
                cvSink.grabFrame(source);
                grip.process(source);
                outputStream.putFrame(source);
            }
        }).start();
    }

}