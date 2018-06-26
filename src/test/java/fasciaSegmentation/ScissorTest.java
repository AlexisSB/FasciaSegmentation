package fasciaSegmentation;

import ij.IJ;
import ij.ImagePlus;
import ij.gui.PolygonRoi;
import ij.gui.StackWindow;
import org.junit.Test;
import weka.gui.treevisualizer.Edge;

import java.awt.*;

import static org.junit.Assert.assertEquals;

public class ScissorTest {

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
        IJ.save(myScissors.imageCost,"/Users/alexis/Anatomy_Project/TestSlices/SaturationTest/Cost.bmp");
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

