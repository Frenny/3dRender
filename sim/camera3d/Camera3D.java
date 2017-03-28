package sim.camera3d;

import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javafx.embed.swing.JFXPanel;
import javafx.scene.paint.Color;
import javafx.scene.shape.Shape3D;
import sim.Bounds;

public class Camera3D extends JFXPanel {
	
	private static final long serialVersionUID = 1L;
	
	private final static int CAMERA_IMAGE_WIDTH = 320;
    private final static int CAMERA_IMAGE_HEIGHT = 240;
    private final static int CAMERA_OFFSET_X = 10;
    private final static int CAMERA_OFFSET_Y = 10;
    private final static int REFRESH_EVERY_MILLISECONDS = 40;
    
    private Bounds parentBounds;
    private CameraSize cameraSize;
    private CameraLocation cameraLocation;
    private ScheduledExecutorService executor;
    private Scene3D scene;
    private World world;
    
	public void resize(Bounds parentBounds, CameraSize cameraSize, CameraLocation cameraLocation) {
		this.parentBounds = parentBounds;
		this.cameraSize = cameraSize;
		this.cameraLocation = cameraLocation;
		Bounds bounds = calculateBounds(parentBounds, cameraSize, cameraLocation);
		super.setBounds(bounds.getX(), bounds.getY(), bounds.getWidth(), bounds.getHeight());
		setOpaque(true);
	}
	
	public void resize(Bounds parentBounds) {
		this.parentBounds = parentBounds;
		resize(parentBounds, cameraSize, cameraLocation);
	}
	
	public void resize(CameraLocation cameraLocation) {
		this.cameraLocation = cameraLocation;
		resize(parentBounds, cameraSize, cameraLocation);
	}
	
	public void resize(CameraSize cameraSize) {
		this.cameraSize = cameraSize;
		resize(parentBounds, cameraSize, cameraLocation);
	}
	
	public void createScene(List<Shape3D> shapes) {
		world = new World();
		world.addShape3D(shapes);	
		
		scene = new Scene3D(world, getWidth(), getHeight());
		setScene(scene);
    }
	
	public void startSimulation() {
		executor = Executors.newSingleThreadScheduledExecutor();
		executor.scheduleAtFixedRate(new Runnable() {
		  @Override
		  public void run() {
			  scene.updateCameraPosition();
		  }
		}, 0, REFRESH_EVERY_MILLISECONDS, TimeUnit.MILLISECONDS);
	}
	
	public void stopSimulation() {
		executor.shutdown();
	}
	
	private Bounds calculateBounds(Bounds parentBounds, CameraSize cameraSize, CameraLocation cameraLocation) {
		int newX = 0;
        int newY = 0;
        int newWidth = 0;
        int newHeight = 0;

        switch (cameraSize) {
           case ORIGINAL:
        	   newWidth = CAMERA_IMAGE_WIDTH;
               newHeight = CAMERA_IMAGE_HEIGHT;
               break;
           case TWICE:
        	   newWidth = CAMERA_IMAGE_WIDTH + CAMERA_IMAGE_WIDTH / 2;
               newHeight = CAMERA_IMAGE_HEIGHT + CAMERA_IMAGE_HEIGHT / 2;
               break;
           case TRIPLE:
        	   newWidth = CAMERA_IMAGE_WIDTH * 2;
        	   newHeight = CAMERA_IMAGE_HEIGHT * 2;
               break;
           case QUAD:
        	   newWidth = CAMERA_IMAGE_WIDTH * 2 + CAMERA_IMAGE_WIDTH / 2;
        	   newHeight = CAMERA_IMAGE_HEIGHT * 2 + CAMERA_IMAGE_HEIGHT / 2;
               break;
           case QUINTUPLE:
        	   newWidth = CAMERA_IMAGE_WIDTH * 3;
        	   newHeight = CAMERA_IMAGE_HEIGHT * 3;
               break;
           case MAX:
        	   newWidth = parentBounds.getWidth();
        	   newHeight = parentBounds.getHeight();
               break; 
           default:
               System.out.println("Illegal Camera Size!");
        }
        
        if (newWidth > parentBounds.getWidth())
        	newWidth = parentBounds.getWidth();
        if (newHeight > parentBounds.getHeight())
        	newHeight = parentBounds.getHeight();
        
        if (newWidth > newHeight * CAMERA_IMAGE_WIDTH / CAMERA_IMAGE_HEIGHT)
        	newWidth = newHeight * CAMERA_IMAGE_WIDTH / CAMERA_IMAGE_HEIGHT;
        else if (newHeight > newWidth * CAMERA_IMAGE_HEIGHT / CAMERA_IMAGE_WIDTH)
        	newHeight= newWidth * CAMERA_IMAGE_HEIGHT / CAMERA_IMAGE_WIDTH;
        
        newX = 0;
        newY = 0;
        
        switch (cameraLocation) {
           case UPPER_LEFT:
        	   newX = CAMERA_OFFSET_X;
               newY = CAMERA_OFFSET_Y;
               break;
           case UPPER_RIGHT:
        	   newX = parentBounds.getWidth() - CAMERA_OFFSET_X - CAMERA_IMAGE_WIDTH;
               newY = CAMERA_OFFSET_Y;
               break;
           case LOWER_LEFT:
        	   newX = CAMERA_OFFSET_X;
               newY = parentBounds.getHeight() - CAMERA_OFFSET_Y - CAMERA_IMAGE_HEIGHT;
               break;
           case LOWER_RIGHT:
        	   newX = parentBounds.getWidth() - CAMERA_OFFSET_X - CAMERA_IMAGE_WIDTH;
               newY = parentBounds.getHeight() - CAMERA_OFFSET_Y - CAMERA_IMAGE_HEIGHT;
               break;
           case CENTER:
        	   newX = (parentBounds.getWidth() - CAMERA_IMAGE_WIDTH) / 2;
               newY = (parentBounds.getHeight() - CAMERA_IMAGE_HEIGHT) / 2;
               break;
           default:
                System.out.println("Illegal Camera Location!");
        }


        if (CAMERA_IMAGE_WIDTH + CAMERA_OFFSET_X * 2 > parentBounds.getWidth()) {
            newX = (parentBounds.getWidth() - CAMERA_IMAGE_WIDTH) / 2;
        }
        if (CAMERA_IMAGE_HEIGHT + CAMERA_OFFSET_Y * 2 > parentBounds.getHeight()) {
            newY = (parentBounds.getHeight() - CAMERA_IMAGE_HEIGHT) / 2;
        }
        
		return new Bounds(newX, newY, newWidth, newHeight);
    }
}
