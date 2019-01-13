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
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {

    private GRIPVision grip = new GRIPVision();

    private int cameraWidth;

    private double errorSum, lastError;

    private long lastTime;

    /**
     * Only run this once!
     */
    public void generateTargetImage(VideoSource camera) {
        new Thread(() -> {

            CvSink cvSink = CameraServer.getInstance().getVideo(camera);
            this.cameraWidth = camera.getVideoMode().width;
            System.out.println("Camera width: " + this.cameraWidth);
            CvSource outputStream = CameraServer.getInstance().putVideo("GRIP", this.cameraWidth,
                    camera.getVideoMode().height);

            Mat source = new Mat();

            while (!Thread.interrupted()) {
                if (cvSink.grabFrame(source) != 0) {
                    grip.process(source);
                    outputStream.putFrame(grip.cvErodeOutput());
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

    public void findAngle() {
        // TODO
    }

	@Deprecated
    public void trackTarget(double error, TalonSRX leftDrive, TalonSRX rightDrive, double kP, double kI, double kD) {

		
        double leftPower, rightPower;

        long currentTime = System.currentTimeMillis() % 1000;

        double p =  (error * kP);
        SmartDashboard.putNumber("Calculated p", p);

        double i = 0.0;
        if (Math.abs(error) < 60) {
            i = (this.errorSum * kI);
        } else {
            this.errorSum = 0;
        }
        SmartDashboard.putNumber("Calculated i", i);

        double d = (((this.lastError - error)/(this.lastTime - currentTime)) * kD);
        SmartDashboard.putNumber("Calculated d", d);

        double power = p + i + d;

        rightPower = power;
        leftPower = -power;

        leftDrive.set(ControlMode.PercentOutput, leftPower);
		rightDrive.set(ControlMode.PercentOutput, rightPower);
		
		if (Math.abs(error) < 60) {
			this.errorSum += (error * (this.lastTime - currentTime));
		} else {
			this.errorSum = 0;
		}
		this.lastError = error;
		this.lastTime = currentTime;
		
    }

	@Deprecated
    private static double checkPower(double currentPower, double maxPower) {
        if (currentPower > maxPower) {
            return maxPower;
        } else if (currentPower < -maxPower) {
            return -maxPower;
        } else {
            return currentPower;
        }
    }

	@Deprecated
    public void resetPID() {
        this.errorSum = 0.0d;
        this.lastError = 0.0d;
	}
	
	public double getDegree(double centerX) {
		return (0.2*centerX)-1;
	}

}