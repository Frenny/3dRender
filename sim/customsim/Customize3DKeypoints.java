package sim.customsim;

import robotinterface.vss.KeyPointPackage3D;

/**
* This interface models customization of the 3D keypoint generation. 
* This is required to model realistic errors that may occur in a real (not simulated) processing chain that derives 3D keypoints from camera images.
* Where as the uncustomized key point generation always produces exact and steady entries of all viewable keypoints, this is not
* possible for real image processing. As a result, a real processing chain usually has filters and mechanisms that detects wrong keypoints and is able to correct position errors.
* To test such mechanisms, the developer/tester may develop an instance of this interface, that is passed to the simulation 
* environment via command line parameter 
* <pre>
*    -custom3d classname
* </pre>
* Customization may be:
* <ul>
*  <li>some viewpoint's 3D position may be randomly modified</li>
*  <li>some keypoints may be removed</li>
*  <li>some keypoints are created as random keypoints</li>
* </ul>
*/
public interface Customize3DKeypoints {


/**
* Customize a list of keypoints (2D).
* @param keypoints3D the exact (&quot;clean&quot;) list of keypoints (3D) is passed and also the return value, i.e. the method modifies this list
* @throws Exception if something happens
*/
     public void customize(KeyPointPackage3D keypoints3D) throws Exception;

}