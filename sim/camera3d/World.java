package sim.camera3d;

import java.util.List;

import javafx.scene.AmbientLight;
import javafx.scene.Group;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Shape3D;

public class World extends Group {
	
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
}
