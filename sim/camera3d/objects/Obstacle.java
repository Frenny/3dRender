package sim.camera3d.objects;

import javafx.scene.paint.Material;
import javafx.scene.shape.CullFace;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;

public class Obstacle extends MeshView {

	double[][] keypoints;
	double height;
	Material material;
	TriangleMesh triangleMesh;
	
	public Obstacle(double[][] keypoints, double height, Material material) {
		this.keypoints = keypoints;
		this.height = height;
		this.material = material;
		
		triangleMesh = new TriangleMesh();
		
		this.setVertices();
		this.setFaces();
		this.setTextureCoordinates();
		
		this.setMesh(triangleMesh);
		this.setCullFace(CullFace.NONE);
		this.setMaterial(this.material);
		this.setTranslateY(-this.height);
	}
	
	private void setVertices() {
		for(int i=0; i < keypoints.length; i++) {
			float x = (float) keypoints[i][0];
			float y = 0;
			float z = (float) keypoints[i][1];
			triangleMesh.getPoints().addAll(x,y,z);
		}
		
		for(int i=0; i < keypoints.length; i++) {
			float x = (float) keypoints[i][0];
			float y = (float) height;
			float z = (float) keypoints[i][1];
			triangleMesh.getPoints().addAll(x,y,z);
		}
	}
	
	private void setFaces() {
		//Boden: n-2 Dreicke nötig, wobei n die Anzahl der Punkte ist
		for(int i=0; i <= keypoints.length-2;i+=2) {
			triangleMesh.getFaces().addAll(i,0, (i+2)%(keypoints.length),0, (i+1),0);
		}
		
		//Seiten: 2*n Dreicke nötig, wobei n die Anzahl der Punkte ist
		for(int i=0; i < keypoints.length-1;i++) {
			triangleMesh.getFaces().addAll((i+keypoints.length),0, (i+keypoints.length+1),0, (i+1),0);
			triangleMesh.getFaces().addAll(i,0, (i+keypoints.length),0, (i+1),0);
		}
		
		//Sonderbehandlung der bei den letzen beiden: Übergang von n zu 0!
		triangleMesh.getFaces().addAll((2*keypoints.length-1),0, keypoints.length,0, 0,0);
		triangleMesh.getFaces().addAll((keypoints.length-1),0, (2*keypoints.length-1),0, 0,0);
		
		//Dach:  n-2 Dreicke nötig, wobei n die Anzahl der Punkte ist
		for(int i=keypoints.length; i < 2*keypoints.length-2;i+=2) {
			triangleMesh.getFaces().addAll(i,0, (i+2),0, (i+1),0);
		}		
		
		//Sonderbehandlung des letzen: Übergang von n zu 0!
		if(keypoints.length != 3) {
			triangleMesh.getFaces().addAll((2*keypoints.length-2),0, keypoints.length,0, (2*keypoints.length-1),0);
		}
	}
	
	private void setTextureCoordinates() {
		triangleMesh.getTexCoords().addAll(0,0);
	}

}
