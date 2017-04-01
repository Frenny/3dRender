package sim.camera3d;

import java.util.ArrayList;
import java.util.List;

import com.vividsolutions.jts.geom.Coordinate;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.CullFace;
import javafx.scene.shape.DrawMode;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Shape3D;
import javafx.scene.shape.TriangleMesh;
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
			Shaddow(shapes, wall);
		}
		
		return shapes;
	}
	
	public static void Shaddow(List<Shape3D> shapes, sim.environment.Wall wall)
	{
		Coordinate[] cor = wall.jtsShadowGeom.getCoordinates();
		System.out.println("Shaddow-Points" + cor.length);
		double[][] shaddowCoords = new double[cor.length][3];
		for(int i =0; i<cor.length; i++)
		{
				shaddowCoords[i][0] = cor[i].x;
				shaddowCoords[i][1] = cor[i].y;
				shaddowCoords[i][2] = cor[i].z;
			
				System.out.println("x: " + cor[i].x + " y: " + cor[i].y + " z: " +cor[i].z);
		}
			
		
		//shapes.add(new Obstacle(shaddowCoords, new PhongMaterial(Color.rgb(50, 50, 50,0.3)), 0.1f));
		TriangleMesh triangleMesh = new TriangleMesh();
		
		//Set texture coordinates
		triangleMesh.getTexCoords().addAll(0,0);
		
		//Set vertices of the obstacles
		for(int i=0; i < shaddowCoords.length; i++) {
			float x = (float) shaddowCoords[i][0];
			float y = 0.1f;
			float z = (float) shaddowCoords[i][1];
			triangleMesh.getPoints().addAll(x,y,z);
		}
		
		List<Integer> hilfe = new ArrayList<Integer>();
		
		for(int i=0; i <= shaddowCoords.length-2;i+=2) {
			hilfe.add(i);
			triangleMesh.getFaces().addAll(i,0, (i+2)%(shaddowCoords.length),0, (i+1),0);
			System.out.println("p1: " + i + " ps2: " + (i+2)%(shaddowCoords.length) + " p3: " + (i+1) );
		}



Integer[] h = hilfe.stream().toArray(Integer[]::new);
		
		while(h.length >= 3)
		{
			hilfe.clear();
			for(int i=0; i <= h.length-2;i+=2) {
				
				hilfe.add(i);
				int a = h[i];
				int b = h[i+1];
				int c;
				if(i+2 > h.length-1)
				{
					c = h[0];
				}
				else
				{
					c = h[i+2];
				}
				triangleMesh.getFaces().addAll(a,0, c,0, b,0);
				System.out.println("p1:" + a +" p2: "+ c +" p3: " + b );
			}
			h = hilfe.stream().toArray(Integer[]::new);
		}
		
		hilfe.clear();
		
		MeshView view = new MeshView(triangleMesh);
		view.setCullFace(CullFace.NONE);
		view.setMaterial(new PhongMaterial(Color.PINK));
		//view.setMaterial(new PhongMaterial(Color.rgb(50, 50, 50,0.3)));
		view.setTranslateY(-1);
		view.setDrawMode(DrawMode.FILL);
		
		
		shapes.add(view);
	}

}
