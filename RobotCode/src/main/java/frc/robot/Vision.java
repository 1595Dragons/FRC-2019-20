package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;

public class Vision {

    private GRIPVision grip = new GRIPVision();

    private int cameraWidth;

    private double sumError, pastError;

    /**
     * Only run this once!
     */
    public void generateTargetImage(VideoSource camera) {
        new Thread(() -> {

            CvSink cvSink = CameraServer.getInstance().getVideo(camera);
            this.cameraWidth = camera.getVideoMode().width;
            CvSource outputStream = CameraServer.getInstance().putVideo("GRIP", this.cameraWidth,
                    camera.getVideoMode().height);

            Mat source = new Mat();

            while (!Thread.interrupted()) {
                if (cvSink.grabFrame(source) != 0) {
                    grip.process(source);
                    outputStream.putFrame(grip.maskOutput());
                }
            }
        }).start();
    }

    public double findCenterX() {

        double centerX = 0.0d;

        // Check if there are contours to go off of
        if (grip.filterContoursOutput.isEmpty()) {
            return 0.0d;
        }

        // Get the number of contours, and find their center X value
        for (MatOfPoint matpoint : grip.filterContoursOutput) {
            Rect r = Imgproc.boundingRect(matpoint);
            centerX = ((r.x + r.width) - (r.width / 2) - (this.cameraWidth / 2));
        }

        return centerX;

    }

    public void trackTarget(double error, TalonSRX leftDrive, TalonSRX rightDrive, double maxPower, double kP,
            double kI, double kD) {

        double leftPower, rightPower;

        double power = (error * kP) + (this.sumError * kI) + ((this.pastError - error) * kD);

        rightPower = power;
        leftPower = -power;

        this.sumError += error;
        this.pastError = error;

        // Manage maxPower
        // TODO: Left motors seem to be grinding against each ther...
        rightPower = Vision.checkPower(rightPower, maxPower);
        leftPower = Vision.checkPower(leftPower, maxPower);

        leftDrive.set(ControlMode.PercentOutput, leftPower);
        rightDrive.set(ControlMode.PercentOutput, rightPower);
    }

    private static double checkPower(double currentPower, double maxPower) {
        if (currentPower > maxPower) {
            return maxPower;
        } else if (currentPower < -maxPower) {
            return -maxPower;
        } else {
            return currentPower;
        }
    }

    public void resetError() {
        this.pastError = 0;
        this.sumError = 0;
    }

}