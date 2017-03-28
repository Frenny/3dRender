package sim.camera3d;

public enum CameraLocation {
	UPPER_LEFT(0), 
	UPPER_RIGHT(1), 
	LOWER_LEFT(2), 
	LOWER_RIGHT(3), 
	CENTER(4);

	private int value;
	
	CameraLocation(int value) {
		this.value = value;
	}
	
	int getValue() {
		return value;
	}
}
