package sim.camera3d;

import javafx.scene.PerspectiveCamera;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.paint.Color;
import javafx.scene.transform.Rotate;
import jr.motion.DCMotorController;
import robotinterface.Robot;

public class Scene3D extends Scene {
	
	private final static double FIELD_OF_VIEW_ANGLE = 42.0;
	private final static double CAMERA_HEIGHT = 25.0; //TODO: Passt noch nicht ganz exakt zum bisherigen Bild
	private final static double PIDIV180 = Math.PI/180.0d;
	
	private PerspectiveCamera camera;
	private Rotate rotate;
	
	public Scene3D(World world, int width, int height) {
		super(world, width, height, true, SceneAntialiasing.BALANCED);
		setFill(Color.LIGHTSTEELBLUE);
		setInitalCamera();
	}
	
	private void setInitalCamera() {
		rotate = new Rotate(0.0, Rotate.Y_AXIS);
		
		camera = new PerspectiveCamera(true);
		camera.setFieldOfView(FIELD_OF_VIEW_ANGLE);
		camera.setNearClip(0.1);
		camera.setFarClip(10000);
		camera.getTransforms().add(rotate);
		
		camera.setTranslateY(-CAMERA_HEIGHT);
		this.updateCameraPosition();
		
        super.setCamera(camera);
    }
	
	public void updateCameraPosition() {
		camera.setTranslateX(this.currentPositionX());
		camera.setTranslateZ(this.currentPositionZ());
		rotate.setAngle(this.currentAngle());
	}
	
	private double currentPositionX() {
        double currentSina = Math.sin(this.currentAngle() * PIDIV180);
        return Robot.camDist * currentSina + DCMotorController.currentPosX;
	}
	
	private double currentPositionZ() {
        double currentCosa = Math.cos(this.currentAngle() * PIDIV180);
        return Robot.camDist * currentCosa + DCMotorController.currentPosY;
	}
	
	private double currentAngle() {
		return DCMotorController.currentAngle;
	}
}
