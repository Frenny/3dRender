package sim.camera3d.objects;

import javafx.scene.paint.Material;
import javafx.scene.shape.Box;

public class Wall extends Box {

	public Wall(double[][] keypoints, Material material, double height) {
	    double maxX= Math.max(Math.max(keypoints[0][0], keypoints[1][0]), Math.max(keypoints[2][0], keypoints[3][0]));
	    double minX = Math.min(Math.min(keypoints[0][0], keypoints[1][0]), Math.min(keypoints[2][0], keypoints[3][0]));
	    double maxZ = Math.max(Math.max(keypoints[0][1], keypoints[1][1]), Math.max(keypoints[2][1], keypoints[3][1]));
	    double minZ = Math.min(Math.min(keypoints[0][1], keypoints[1][1]), Math.min(keypoints[2][1], keypoints[3][1]));
	    
		double width = Math.abs(maxX-minX);
		double depth = Math.abs(maxZ-minZ);
		
		this.setWidth(width);
		this.setHeight(height);
		this.setDepth(depth);
	    
		this.setTranslateX(minX + 0.5 * width);
		this.setTranslateY(-(0.5 * height));
		this.setTranslateZ(minZ + 0.5 * depth);
		
		this.setMaterial(material);
	}
}
