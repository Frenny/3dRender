package sim.customsim;

import robotinterface.vss.KeyPointPackage2D;

/**
* This interface models customization of the 2D keypoint generation. 
* This is required to model realistic errors that may occur in a real (not simulated) processing chain that derives 2D keypoints from camera images.
* Where as the uncustomized key point generation always produces exact and steady entries of all viewable keypoints, this is not
* possible for real image processing. As a result, a real processing chain usually has filters and mechanisms that detects wrong keypoints and is able to correct position errors.
* To test such mechanisms, the developer/tester may develop an instance of this interface, that is passed to the simulation 
* environment via command line parameter 
* <pre>
*    -custom2d classname
* </pre>
* Customization may be:
* <ul>
*  <li>some viewpoint's pixel position may be randomly modified</li>
*  <li>some keypoints may be removed</li>
*  <li>some keypoints are created as random keypoints</li>
*  <li>the line ID of some keypoints may be randomly modified to new (non-existant) values</li>
*  <li>the line ID of some keypoints may be exchanged among each others</li>
* </ul>
*/
public interface Customize2DKeypoints {


/**
* Customize a list of keypoints (2D).
* @param keypoints2D the exact (&quot;clean&quot;) list of keypoints (2D) is passed and also the return value, i.e. the method modifies this list
* @throws Exception if something happens
*/
     public void customize(KeyPointPackage2D keypoints2D) throws Exception;

}