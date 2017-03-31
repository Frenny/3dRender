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
import sim.camera3d.objects.Track;

public class World extends Group {
	
	private ArrayList<Track> tyreTrackFrontLeft = new ArrayList<>();
	private ArrayList<Track> tyreTrackFrontRight = new ArrayList<>();
	private ArrayList<Track> tyreTrackBackLeft = new ArrayList<>();
	private ArrayList<Track> tyreTrackBackRight = new ArrayList<>();
	
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
		addTrackIfNeeded(SimCanvas.tyreTrackFrontLeft, tyreTrackFrontLeft, SimCanvas.MAX_TRACK_ENTRIES);
		addTrackIfNeeded(SimCanvas.tyreTrackFrontRight, tyreTrackFrontRight, SimCanvas.MAX_TRACK_ENTRIES);
		addTrackIfNeeded(SimCanvas.tyreTrackBackLeft, tyreTrackBackLeft, SimCanvas.MAX_TRACK_ENTRIES);
		addTrackIfNeeded(SimCanvas.tyreTrackBackRight, tyreTrackBackRight, SimCanvas.MAX_TRACK_ENTRIES);
	}
	
	private void addTrackIfNeeded(final List<double[]> simCanvasTrack, final List<Track> tyreTrack, final int maxTrackEntries) {
		if (tyreTrack.size() >= maxTrackEntries) {
			getChildren().remove(tyreTrack.get(0));
			tyreTrack.remove(0);
		}
		
		int lastIndex = simCanvasTrack.size() - 1;
		int lastTyreIndex = tyreTrack.size() - 1;
		
		if(lastIndex >= 0) {
			double[] lastPoint = simCanvasTrack.get(lastIndex);
			if(lastTyreIndex < 0) {
				Track track = new Track(lastPoint);
				tyreTrack.add(track);
				addShape3D(track);
			} else {
				Track lastTrack = tyreTrack.get(lastTyreIndex);
				if(lastTrack.getKeypoint() != lastPoint) {
					Track track = new Track(lastPoint);
					tyreTrack.add(track);
					addShape3D(track);
				}
			}
		}
	}
	
	public void clearTrack() {
		getChildren().removeAll(tyreTrackFrontLeft);
		getChildren().removeAll(tyreTrackFrontRight);
		getChildren().removeAll(tyreTrackBackLeft);
		getChildren().removeAll(tyreTrackBackRight);
		tyreTrackFrontLeft.clear();
		tyreTrackFrontRight.clear();
		tyreTrackBackLeft.clear();
		tyreTrackBackRight.clear();
	}
}
