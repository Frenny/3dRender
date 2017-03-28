package sim.camera3d.objects;

import java.util.Arrays;

import javafx.scene.paint.Material;
import javafx.scene.shape.CullFace;
import javafx.scene.shape.DrawMode;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;

public class Obstacle extends MeshView {
	
	public Obstacle(double[][] keypoints, Material material, double height) {
		
		TriangleMesh triangleMesh = new TriangleMesh();
			
		//Set texture coordinates
		triangleMesh.getTexCoords().addAll(0,0);
		
		//Set vertices of the obstacles
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
		
		
		//Create the faces
		//Sample Dreickeck:
		// 0,0, 2,0, 1,0 => Boden
		// 3,0, 4,0, 1,0 
		// 0,0, 3,0, 1,0
		// 4,0, 5,0, 2,0
		// 1,0, 4,0, 2,0
		// 5,0, 3,0, 0,0      (Sonderbehandlung wegen mittlerem Element und letztem!)
		// 2,0, 5,0, 3,0
		// 3,0, 5,0, 4,0 => Dach (Sonderbehandlung)
		
		//Sample Viereck:
		//0,0, 2,0, 1,0 => Boden
		//2,0, 0,0, 3,0 => Boden
		//4,0, 5,0, 1,0
		//0,0, 4,0, 1,0
		//5,0, 6,0, 2,0
		//1,0, 5,0, 2,0
		//6,0, 7,0, 3,0
		//2,0, 6,0, 3,0
		//7,0, 4,0, 0,0 (Sonderbehandlung wegen mittlerem Element und letztem!)
		//3,0, 7,0, 0,0
		//4,0, 6,0, 5,0 => Dach (Sonderbehandlung)
		//6,0, 4,0, 7,0 => Dach (Sonderbehandlung)
		
		//Boden: n-2 Dreicke n�tig, wobei n die Anzahl der Punkte ist
		for(int i=0; i <= keypoints.length-2;i+=2) {
			triangleMesh.getFaces().addAll(i,0, (i+2)%(keypoints.length),0, (i+1),0);
		}

		
		//Seiten: 2*n Dreicke n�tig, wobei n die Anzahl der Punkte ist
		for(int i=0; i < keypoints.length-1;i++) {
			triangleMesh.getFaces().addAll((i+keypoints.length),0, (i+keypoints.length+1),0, (i+1),0);
			triangleMesh.getFaces().addAll(i,0, (i+keypoints.length),0, (i+1),0);
		}
		
		//Sonderbehandlung der bei den letzen beiden: �bergang von n zu 0!
		triangleMesh.getFaces().addAll((2*keypoints.length-1),0, keypoints.length,0, 0,0);
		triangleMesh.getFaces().addAll((keypoints.length-1),0, (2*keypoints.length-1),0, 0,0);
		
		//Dach:  n-2 Dreicke n�tig, wobei n die Anzahl der Punkte ist
		for(int i=keypoints.length; i < 2*keypoints.length-2;i+=2) {
			triangleMesh.getFaces().addAll(i,0, (i+2),0, (i+1),0);
		}		
		
		//Sonderbehandlung des letzen: �bergang von n zu 0!
		if(keypoints.length != 3) {
			triangleMesh.getFaces().addAll((2*keypoints.length-2),0, keypoints.length,0, (2*keypoints.length-1),0);
		}
		
		
		this.setMesh(triangleMesh);
		this.setCullFace(CullFace.NONE); //verhindert das Flackern der Mesh-Objekte
		this.setMaterial(material);
		this.setTranslateY(-height); //H�he anpassen
		

	}

}
