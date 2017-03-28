package sim.camera3d;

import java.util.ArrayList;
import java.util.List;

import javafx.scene.AmbientLight;
import javafx.scene.Group;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.Shape3D;
import sim.SimCanvas;

public class World extends Group {
	
	private ArrayList<Cylinder> trackPoints = new ArrayList<>();
	private ArrayList<double[]> tyreTrackFrontLeft = new ArrayList<>();
	private ArrayList<double[]> tyreTrackFrontRight = new ArrayList<>();
	private ArrayList<double[]> tyreTrackBackLeft = new ArrayList<>();
	private ArrayList<double[]> tyreTrackBackRight = new ArrayList<>();
	
	public World() {
		super();
		//addSky();
		addGround();
	}
	
	private void addSky() {
		Box sky = new Box(100000,0,100000);
		sky.setTranslateY(-100);
		
		PhongMaterial mat = new PhongMaterial();
		mat.setDiffuseMap(new Image(getClass().getResourceAsStream("/sky.jpg")));
		sky.setMaterial(mat);
		
		getChildren().addAll(sky, new AmbientLight());
	}
	
	private void addGround() {
		Box ground = new Box(10000,0,10000);
		ground.setTranslateY(0);
		
		/*PhongMaterial mat = new PhongMaterial();
		mat.setDiffuseMap(new Image(getClass().getResourceAsStream("/ground.jpg")));
		ground.setMaterial(mat);*/
		
		ground.setMaterial(new PhongMaterial(Color.WHITE));
		
		getChildren().addAll(ground, new AmbientLight());
	}
	
	public void addShape3D(Shape3D shape) {
		getChildren().add(shape);//, new AmbientLight());
	}
	
	public void addShape3D(List<Shape3D> shapes) {
		for(Shape3D shape : shapes) {
	    	this.addShape3D(shape);
	    }
	}
	
	public void refreshTrack() {
		for(double[] point : SimCanvas.tyreTrackFrontLeft) {
			if(!tyreTrackFrontLeft.contains(point)) {
				addTrack(point);
				tyreTrackFrontLeft.add(point);
			}
		}
		for(double[] point : SimCanvas.tyreTrackFrontRight) {
			if(!tyreTrackFrontRight.contains(point)) {
				addTrack(point);
				tyreTrackFrontRight.add(point);
			}
		}
		for(double[] point : SimCanvas.tyreTrackBackLeft) {
			if(!tyreTrackBackLeft.contains(point)) {
				addTrack(point);
				tyreTrackBackLeft.add(point);
			}
		}
		for(double[] point : SimCanvas.tyreTrackBackRight) {
			if(!tyreTrackBackRight.contains(point)) {
				addTrack(point);
				tyreTrackBackRight.add(point);
			}
		}
	}
	
	private void addTrack(double[] point) {
		Cylinder trackPoint = new Cylinder(1, 0);
		trackPoint.setTranslateX(point[0]);
		trackPoint.setTranslateY(-1);
		trackPoint.setTranslateZ(point[1]);
		
		trackPoint.setMaterial(new PhongMaterial(Color.LIGHTGRAY));
		
		trackPoints.add(trackPoint);
		addShape3D(trackPoint);
	}
	
	public void clearTrack() {
		getChildren().removeAll(trackPoints);
		trackPoints.clear();
		tyreTrackFrontLeft.clear();
		tyreTrackFrontRight.clear();
		tyreTrackBackLeft.clear();
		tyreTrackBackRight.clear();
	}
}
