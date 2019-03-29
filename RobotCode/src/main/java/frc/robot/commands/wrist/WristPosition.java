package frc.robot.commands.wrist;

public enum WristPosition {
	UP(-3087), ZERO(-2063), MINUS180(-4111), FORWARDEXCHANGE(-2887), REVERSEEXCHANGE(-3287);

	private int value;

	public int getValue() {
		return this.value;
	}

	private WristPosition(int value) {
		this.value = value;
	}
}