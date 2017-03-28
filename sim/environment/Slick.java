package sim.environment;

import java.util.ArrayList;

/**
* An area that reduces the friction of wheels. As a result, the robot 'thinks' the movement is greater than in reality.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class Slick extends Region {


     public double slickLeft=1.0d;    // Rutschigkeitswert: 1.0: Räder greifen exakt, 2.0: Robot bewegt sich nur halb so weit, wie die Räder drehen
     public double slickRight=1.0d; 


/**
* Creates a lick region.
* @param name name for display on the map.
* @param slickLeft slick value for left tyre (usually &gt;0): e.g. 2.0: Robot moves half the distance than on normal pavement.
* @param slickRight slick value for right tyre 
* @param positions the positions (shell of the area, no holes)
*/
     public Slick(String name,double slickLeft,double slickRight,ArrayList<double[]> positions) {
         super(name,positions,0.0d,false);
         this.slickLeft=slickLeft;
         this.slickRight=slickRight;
     }

}