package sim.camera3d.objects;

import javafx.scene.paint.Material;

public class Cylinder extends javafx.scene.shape.Cylinder {
	
	public Cylinder(double[][] keypoints, Material material, double height) {
		double x = keypoints[5][0];
		double z = keypoints[0][1];
		double radius = keypoints[0][0] - x;
		
		this.setRadius(radius);
		this.setHeight(height);
	    
		this.setTranslateX(x);
		this.setTranslateY(-(0.5 * height));
		this.setTranslateZ(z);
		
		this.setMaterial(material);
	}
}
