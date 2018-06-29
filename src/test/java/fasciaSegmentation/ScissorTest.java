package fasciaSegmentation;

import ij.IJ;
import ij.ImagePlus;
import ij.gui.PolygonRoi;
import org.junit.Before;
import org.junit.Test;

import java.awt.*;
import java.util.ArrayList;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class ScissorTest {

    private IntelligentScissorsTwo scissors;
    ImagePlus testImage;

    @Before
    public void setUp(){
        scissors = new IntelligentScissorsTwo();
        testImage = IJ.openImage("/Users/alexis/Anatomy_Project/FasciaSegmentation/src/test/testpics/2.bmp");
        assertTrue(testImage != null);
        scissors.setImage(testImage);
    }

    @Test
    public void testSetImage(){

        assertTrue("Centre cost matrix uninitialised", scissors.centreLineCostMatrix != null);
        assertTrue("Edge cost matrix uninitialised", scissors.edgeCostMatrix !=null);

        IJ.save(scissors.centreLineCostMatrix, "/Users/alexis/Anatomy_Project/FasciaSegmentation/src/test/testpics/centrecost.bmp");
        IJ.save(scissors.edgeCostMatrix, "/Users/alexis/Anatomy_Project/FasciaSegmentation/src/test/testpics/edgecost.bmp");

    }

    @Test
    public void testSettingUserSelectedPoints(){

        ArrayList<Point> points = new ArrayList<Point>();
        points.add(new Point(10,10));
        points.add(new Point( 20,10));
        scissors.setUserSelectedPoints(points);

        assertFalse(scissors.centreLinePath.isEmpty());
        assertFalse(scissors.edgeLinePath.isEmpty());
        assertFalse(scissors.oppositeEdgeLinePath.isEmpty());;


    }

    @Test
    public void testSetUpPathSearchMatrix(){
        scissors.setUpPathSearchMatrix();
        assertTrue("pathSearchMatrix uninitialised", scissors.pathSearchMatrix !=null);

        for(int row = 0 ; row < testImage.getHeight(); row++){
            for( int col = 0; col < testImage.getWidth(); col++){
                assertTrue("State not set to initial state" , scissors.pathSearchMatrix[row][col].state == IntelligentScissorsTwo.State.INITIAL);
                assertTrue("Previous node not set to null" , scissors.pathSearchMatrix[row][col].previous == null);
                assertTrue("Cost not set to double max" , scissors.pathSearchMatrix[row][col].cost == Double.MAX_VALUE);
                assertTrue("X coordinate incorrect", scissors.pathSearchMatrix[row][col].x == col);
                assertTrue("Y coordinate incorrect", scissors.pathSearchMatrix[row][col].y == row);
            }
        }
    }

    @Test
    public void testFindNormalDirection(){
        ArrayList<Point> path = new ArrayList<Point>();

        path.add(new Point (0,0));
        path.add(new Point (3,9));
        path.add(new Point (6,36));

        double slope = scissors.findNormalDirection(path);

        //System.out.println(slope);
        assertEquals(slope , (-1.0/6.0), 0);

    }


    @Test
    public void testShortestPath(){
        //Grayscale image with no saturation
        ImagePlus image = IJ.openImage("/Users/alexis/Anatomy_Project/FasciaSegmentation/ImageJIntelligentScissors/StripTest.bmp");

        IntelligentScissors myScissors = new IntelligentScissors();
        myScissors.setImage(image);
        Point[] path = new  Point[2];
        Point one = new Point(10,60);
        Point two = new Point(20,60);
        path[0] = one;
        path[1] = two;
        PolygonRoi line = (PolygonRoi) myScissors.drawShortestPath(path);

        System.out.println("Roi line : \n" + line);
    }
    @Test
    public void testBottomSelectBug() {

        //Test the scissor algorithm on the bottom part of the image.
        ImagePlus image = IJ.openImage("/Users/alexis/Anatomy_Project/Working_Images/300DPI_renamed/300DPI00.bmp");
        IntelligentScissors myScissors = new IntelligentScissors();
        myScissors.setImage(image);
        Point[] path = new  Point[2];
        Point one = new Point(10,720);
        Point two = new Point(100,720);
        path[0] = one;
        path[1] = two;
        PolygonRoi line = (PolygonRoi) myScissors.drawShortestPath(path);
        image.setRoi(line);
        System.out.println("Roi line : \n" + line);

        one = new Point(1000,1000);
        two = new Point(1010,1000);
        path[0] = one;
        path[1] = two;
        line = (PolygonRoi) myScissors.drawShortestPath(path);
        image.setRoi(line);
        System.out.println("Roi line : \n" + line);

        one = new Point(100,2500);
        two = new Point(110,2500);
        path[0] = one;
        path[1] = two;
        line = (PolygonRoi) myScissors.drawShortestPath(path);
        image.setRoi(line);
        System.out.println("Roi line : \n" + line);

        one = new Point(100,2513);
        two = new Point(110,2513);
        path[0] = one;
        path[1] = two;
        line = (PolygonRoi) myScissors.drawShortestPath(path);
        image.setRoi(line);
        System.out.println("Roi line : \n" + line);
     }

    @Test
    public void testEdgeScissor(){
        ImagePlus image = IJ.openImage("/Users/alexis/Anatomy_Project/TestSlices/SaturationTest/3.bmp");
        EdgeScissors myScissors= new EdgeScissors();
        myScissors.setImage(image);
        Point[] path = new  Point[2];
        Point one = new Point(10,10);
        Point two = new Point(20,10);
        path[0] = one;
        path[1] = two;
        PolygonRoi line = (PolygonRoi) myScissors.drawShortestPath(path);
        image.setRoi(line);
        image = image.flatten();
        IJ.save(image, "/Users/alexis/Anatomy_Project/TestSlices/SaturationTest/Test.bmp");
        //IJ.save(myScissors.imageCost,"/Users/alexis/Anatomy_Project/TestSlices/SaturationTest/Cost.bmp");
        System.out.println("Roi line : \n" + line);

    }

    @Test
    public void testInfoMatrixSetup(){
        ImagePlus image = IJ.openImage("/Users/alexis/Anatomy_Project/FasciaSegmentation/ImageJIntelligentScissors/StripTest.bmp");
        IntelligentScissors myScissors = new IntelligentScissors();
        myScissors.setImage(image);

        for(int i = 0; i < image.getHeight(); i++){
            for(int j = 0; j < image.getWidth(); j++){
                assertEquals(i, myScissors.infoMatrix[i][j].getPoint().y);
                assertEquals(j, myScissors.infoMatrix[i][j].getPoint().x);
            }
        }


    }


}

