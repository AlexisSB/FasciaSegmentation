package fasciaSegmentation;


import java.awt.*;
import java.util.ArrayList;

/**
 *  This class draws the outer edges of the fascia to find the thickness of important structures
 *  Use active contours approach to find and follow the edge.
 */
@Deprecated
public class RibbonSnake {

    ArrayList<Point> left = null;
    ArrayList<Point> middle = null;
    ArrayList<Point> right = null;

    public RibbonSnake(){
        left = new ArrayList<Point>();
        right = new ArrayList<Point>();
        middle = new ArrayList<Point>();
    }

    public void setMiddle(final ArrayList<Point> middle){
        assert(middle != null);
        this.middle = middle;
        //calculate left and right
    }

    public void calculateLeftAndRight(){
        assert(middle != null);

        //for each point in middle, calculate perpendicular point to the line.

    }

}
