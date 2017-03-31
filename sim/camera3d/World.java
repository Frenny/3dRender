package sim.camera3d;

import java.util.ArrayList;
import java.util.List;

import javafx.scene.AmbientLight;
import javafx.scene.Group;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Shape3D;
import sim.SimCanvas;
import sim.camera3d.objects.Track;

public class World extends Group {
	
	List<Track> tyreTrackFrontLeft = new ArrayList<>();
	List<Track> tyreTrackFrontRight = new ArrayList<>();
	List<Track> tyreTrackBackLeft = new ArrayList<>();
	List<Track> tyreTrackBackRight = new ArrayList<>();
	
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
	
	public void refreshTracks() {
		addTrack(SimCanvas.tyreTrackFrontLeft, tyreTrackFrontLeft);
		addTrack(SimCanvas.tyreTrackFrontRight, tyreTrackFrontRight);
		addTrack(SimCanvas.tyreTrackBackLeft, tyreTrackBackLeft);
		addTrack(SimCanvas.tyreTrackBackRight, tyreTrackBackRight);
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
	
	private void addTrack(final List<double[]> simCanvasTrack, final List<Track> tyreTrack) {
		if (tyreTrack.size() >= SimCanvas.MAX_TRACK_ENTRIES) {
			getChildren().remove(tyreTrack.get(0));
			tyreTrack.remove(0);
		}
		
		int lastTrackIndex = simCanvasTrack.size() - 1;
		int lastDrawedTrackIndex = tyreTrack.size() - 1;
		
		if(lastTrackIndex >= 0) {
			double[] position = simCanvasTrack.get(lastTrackIndex);
			if(lastDrawedTrackIndex >= 0) {
				if(tyreTrack.get(lastDrawedTrackIndex).getKeypoint() != position) {
					drawTrack(position, tyreTrack);
				}
			} else {
				drawTrack(position, tyreTrack);
			}
		}
	}
	
	private void drawTrack(double[] position, final List<Track> tyreTrack) {
		Track track = new Track(position);
		tyreTrack.add(track);
		addShape3D(track);
	}
}
