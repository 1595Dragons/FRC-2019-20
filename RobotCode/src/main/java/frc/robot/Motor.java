package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * This is a custom class I made that takes a standard TalonSRX motor, and adds
 * a bunch of funcitons on top of it for more ease of use.
 * 
 * All the standard talonSRX methods still apply and work, its just that there
 * are a few more to choose form now.
 * 
 * @author Stephen - FRC 1595
 */
@Deprecated
public class Motor extends com.ctre.phoenix.motorcontrol.can.TalonSRX {

	/**
	 * PID function for the motor, set the P to 1, and everything else to 0. This
	 * can be changed later.
	 */
	private MiniPID pid = new MiniPID(1, 0, 0);

	/**
	 * Target encoder position (in ticks) used in <code>driveToPosition</code>.
	 */
	private double target;

	/**
	 * Constructor for the motor object (custom TalonSRX object).
	 * 
	 * @param port The motor port (ID on the CAN bus)
	 */
	public Motor(int port) {
		super(port);
		this.pid.setOutputLimits(-1, 1);
	}

	/**
	 * Set the P value for the motors PID.
	 * 
	 * @param p The P value.
	 */
	public void setP(double p) {
		this.pid.setP(p);
	}

	/**
	 * Set the I value for the motors PID.
	 * 
	 * @param i The I value.
	 */
	public void setI(double i) {
		this.pid.setI(i);
	}

	/**
	 * Set the D value for the motors PID.
	 * 
	 * @param d The D value.
	 */
	public void setD(double d) {
		this.pid.setD(d);
	}

	/**
	 * Sets the PID values for the motor.
	 * 
	 * @param p The P value.
	 * @param i The I value.
	 * @param d The D value.
	 */
	public void setPID(double p, double i, double d) {
		this.setP(p);
		this.setI(i);
		this.setD(d);
	}

	/**
	 * Checks if the current motor's position has reached its set target position.
	 * 
	 * @return True if the motors current position equals the target position.
	 *         Otherwise it returns false.
	 */
	public boolean targetReached() {
		return this.target == this.getPosition();
	}

	public boolean targetReached(int error) {
		return Math.abs(this.target - this.getPosition()) <= error;
	}

	/**
	 * This will attempt to drive to a provided encoder position using a PID.
	 */
	public void driveToPosition(double position) {
		double power = this.pid.getOutput(this.getPosition(), position);
		this.setPower(power);
	}

	/**
	 * Sets the motor output to 0, essentially acting as a break.
	 */
	public void stop() {
		this.set(ControlMode.PercentOutput, 0.0d);
	}

	/**
	 * Sets the power of the motor to whatever percentage (0 to 1) was provided. A
	 * negative number will result in the motor driving in reverse.
	 * 
	 * @param power The power percentage to supply to the motor (0 to 1)
	 */
	public void setPower(double power) {
		if (power > 1.0d) {
			power = 1.0d;
		} else if (power < -1.0d) {
			power = 1.0d;
		}

		this.set(ControlMode.PercentOutput, power);
	}

	/**
	 * Returns the motor's current encoder position (in ticks).
	 * 
	 * @return The current encoder position.
	 */
	public double getPosition() {
		return ((double) this.getSelectedSensorPosition());
	}

	public void zeroEncoder() throws EncoderError {
		if (!this.setSelectedSensorPosition(0, 0, 0).equals(com.ctre.phoenix.ErrorCode.OK)) {
			throw new EncoderError("Cannot zero encoder. Does this motor not have one?");
		}
	}

	public double getTarget() {
		return this.target;
	}

	public class EncoderError extends Exception {
		static final long serialVersionUID = 10252;

		public EncoderError(String message) {
			super(message);
		}
	}
}