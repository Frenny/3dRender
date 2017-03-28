package sim.camera3d;

public enum CameraSize {
    ORIGINAL(0),
    TWICE(1),
    TRIPLE(2),
    QUAD(3),
    QUINTUPLE(4),
    MAX(5);
	
	private int value;
	
	CameraSize(int value) {
		this.value = value;
	}
	
	int getValue() {
		return value;
	}
}