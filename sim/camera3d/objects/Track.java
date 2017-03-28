package sim.camera3d.objects;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;

public class Track extends javafx.scene.shape.Cylinder {
	
	private final static int TRACK_RADIUS = 1;
	private final static int TRACK_HEIGHT = 0;
	
	private double[] keypoint;
	
	public Track(double[] keypoint) {
		super(TRACK_RADIUS, TRACK_HEIGHT);
		
		this.setKeypoint(keypoint);
		
		this.setTranslateX(keypoint[0]);
		this.setTranslateY(-1);
		this.setTranslateZ(keypoint[1]);
		
		this.setMaterial(new PhongMaterial(Color.LIGHTGRAY));
	}

	public double[] getKeypoint() {
		return keypoint;
	}

	public void setKeypoint(double[] keypoint) {
		this.keypoint = keypoint;
	}
}

