package sim.camera3d;

import java.util.ArrayList;
import java.util.List;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Shape3D;
import sim.camera3d.objects.Cylinder;
import sim.camera3d.objects.Obstacle;
import sim.environment.Environment;

public class Environment3D {
	public static List<Shape3D> getShapes() {
		List<Shape3D> shapes = new ArrayList<Shape3D>();
		Shape3D shape;
		
		for(sim.environment.Wall wall : Environment.walls) {
			if(wall.type == 0) {
				shape = new Obstacle(wall.positions, wall.height, new PhongMaterial(Color.DARKRED));
			} else if(wall.type == 2) {
				if(wall.positions.length == 20) {
					shape = new Cylinder(wall.positions, wall.height, new PhongMaterial(Color.LIMEGREEN));
				} else {
					shape = new Obstacle(wall.positions, wall.height, new PhongMaterial(Color.LIMEGREEN));
				}
			} else {
				shape = new Obstacle(wall.positions, wall.height, new PhongMaterial(Color.GRAY));
			}
			
			shapes.add(shape);
		}
		
		return shapes;
	}
}
