package fasciaSegmentation;

import ij.IJ;
import ij.ImagePlus;
import ij.gui.PolygonRoi;
import ij.measure.CurveFitter;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;


public class CurveFitTest {

    @Test
    public void testCurveFittingBasic(){
        ImagePlus image = IJ.openImage("/Users/alexis/Anatomy_Project/TestSlices/SaturationTest/3.bmp");
        IntelligentScissors myScissors= new IntelligentScissors();
        myScissors.setImage(image);
        ArrayList<Point> path = new ArrayList<Point>();

        Point one = new Point(130,700);
        Point two = new Point(350,650);
        Point three = new Point(530, 700);

        path.add(one);
        path.add(two);
        path.add(three);

        Point[] pathArray = new Point[path.size()];
        path.toArray(pathArray);
        PolygonRoi line = (PolygonRoi) myScissors.drawShortestPath(pathArray);

        image.setRoi(line);
        image = image.flatten();
        IJ.save(image, "/Users/alexis/Anatomy_Project/TestSlices/SaturationTest/Test.bmp");
        IJ.save(myScissors.imageCost,"/Users/alexis/Anatomy_Project/TestSlices/SaturationTest/Cost.bmp");
        System.out.println("Roi line : \n" + line);

        Point[] linePoints = line.getContainedPoints();

        double[] xData = new double[path.size()];
        double[] yData = new double[path.size()];

        //Go through all the points and do interpolation for three points
        //Start one from the start, end one from the end
        //Only need to do this for the selected points?
        for (int i = 0; i < pathArray.length ; i++) {
            xData[i] = pathArray[i].x;
            yData[i] = pathArray[i].y;
        }

        System.out.println("xData : " + Arrays.toString(xData));
        System.out.println("yData : " + Arrays.toString(yData));

        double slope = getPerpendicularSlope(xData, yData);
        System.out.println("Slope : " + slope);

        Point direction = getQuadrantDirection(slope);

        System.out.println(direction);
    }

    @Test
    public void testCurveFittingMultiplePoints(){
        //Load image and setup scissors
        ImagePlus image = IJ.openImage("/Users/alexis/Anatomy_Project/TestSlices/SaturationTest/3.bmp");
        IntelligentScissors myScissors= new IntelligentScissors();
        myScissors.setImage(image);

        //Construct path
        ArrayList<Point> path = new ArrayList<Point>();
        Point one = new Point(100,100);
        Point two = new Point(170,193);
        Point three = new Point(240, 164);
        Point four = new Point(280, 100);
        Point five = new Point(300, 66);
        Point six = new Point(430, 50);
        Point seven = new Point(460,100);

        path.add(one);
        path.add(two);
        path.add(three);
        path.add(four);
        path.add(five);
        path.add(six);
        path.add(seven);

        Point[] pathArray = new Point[path.size()];
        path.toArray(pathArray);
        PolygonRoi line = (PolygonRoi) myScissors.drawShortestPath(pathArray);

        //Calculate angle for each point and the correspondin grid direction
        double[] xData = new double[3];
        double[] yData = new  double[3];

        for (int i = 1 ; i < path.size()-1; i++){
            xData[0] = pathArray[i-1].x;
            xData[1] = pathArray[i].x;
            xData[2] = pathArray[i+1].x;
            yData[0] = pathArray[i-1].y;
            yData[1] = pathArray[i].y;
            yData[2] = pathArray[i+1].y;

            double slope =getPerpendicularSlope(xData,yData);
            Point direction = getQuadrantDirection(slope);

            System.out.println(" Point : " + pathArray[i].x + ", " + pathArray[i].y + " . Slope : " + slope + " . Direction : " + direction);
            System.out.println();

        }
    }


    public static double getPerpendicularSlope(double[] xData, double[] yData){

        CurveFitter curveFitter = new CurveFitter(xData,yData);

        double perpendicularSlope;
        double[] params;
        if(xData.length < 3) {
            curveFitter.doFit(CurveFitter.STRAIGHT_LINE);
            params = curveFitter.getParams();
            System.out.println("Parameters: " + Arrays.toString(params));
            if(params[1] == 0){
                perpendicularSlope = 10000000; //Large number.

            }else{
                perpendicularSlope = -1/params[1];
                System.out.println("Perpendicular Slope : " + perpendicularSlope);
            }

        }else { //Multi point do a quadratic interpolation

            curveFitter.doFit(CurveFitter.POLY2);
            params = curveFitter.getParams();

            System.out.println("Parameters : " + Arrays.toString(params));
            double slope = params[1] + 2 * params[2] * xData[1];
            System.out.println("Poly slope : " + slope);

            //Start here
            perpendicularSlope = -1/slope;


        }

        return perpendicularSlope;

    }


    @Test
    public void testGetQuadrantDirection(){

        double slope;
        Point direction;

        slope = 10;
        direction = getQuadrantDirection(slope);
        assertEquals(direction,new Point(0,-1));

        slope = 2;
        direction = getQuadrantDirection(slope);
        assertEquals(direction, new Point(1,-1));

        slope = 0;
        direction = getQuadrantDirection(slope);
        assertEquals(direction, new Point(1,0));

        slope = -2;
        direction = getQuadrantDirection(slope);
        assertEquals(direction, new Point(1,1));

        slope = -10;
        direction = getQuadrantDirection(slope);
        assertEquals(direction, new Point(0,1));

        slope = 0.8555;
        direction = getQuadrantDirection(slope);
        assertEquals(direction, new Point(1,-1));
    }



    public static Point getQuadrantDirection(double slope){

        double angle = Math.atan(slope);
        angle = Math.toDegrees(angle);
        System.out.println("Angle : " + angle);

        if (angle >= -18 && angle <= 18){
            return new Point(1,0);
        }
        if (angle >= 18 && angle <= 70){
            return new Point(1,-1);
        }
        if (angle >= 70 && angle <= 90){
            return new Point(0,-1);
        }
        if (angle <= -18 && angle >= -70){
            return new Point(1,1);
        }
        if (angle <= -70 && angle >= -90){
            return new Point(0,1);
        }
        return new Point(0,0);
    }
}
