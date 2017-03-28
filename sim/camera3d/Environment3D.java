package sim.camera3d;

import java.util.ArrayList;
import java.util.List;

import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.CullFace;
import javafx.scene.shape.DrawMode;
import javafx.scene.shape.Shape3D;
import sim.camera3d.objects.Cylinder;
import sim.camera3d.objects.Obstacle;
import sim.camera3d.objects.Wall;
import sim.environment.Environment;

public class Environment3D {
	public static List<Shape3D> getShapes() {
		List<Shape3D> shapes = new ArrayList<Shape3D>();
		
		for(sim.environment.Wall wall : Environment.walls) {
			if(wall.type == 0) {
					shapes.add(new Obstacle(wall.positions, new PhongMaterial(Color.DARKRED), wall.height));
			} else if(wall.type == 1) {
				//TODO: Flat .. Was ist das �berhaupt?
			} else if(wall.type == 2) {
				if(wall.positions.length == 20) {
					shapes.add(new Cylinder(wall.positions, new PhongMaterial(Color.LIMEGREEN), wall.height));
				} else {
        			shapes.add(new Obstacle(wall.positions, new PhongMaterial(Color.LIMEGREEN), wall.height));
				}
			} else if(wall.type == 3) {
				if(wall.positions.length == 4) {
					PhongMaterial material = new PhongMaterial();
					//Image texture = new Image("hedge.jpg");
					//material.setDiffuseMap(texture);
					material.setDiffuseColor(Color.GRAY);
					//wallObject.setDrawMode(DrawMode.LINE);
					
					shapes.add(new Obstacle(wall.positions, new PhongMaterial(Color.GRAY), wall.height));
				}
			}
		}
		
		return shapes;
	}
}
