package fasciaSegmentation;

import ij.IJ;
import ij.ImagePlus;
import ij.gui.PolygonRoi;
import ij.measure.CurveFitter;
import org.junit.Test;

import java.awt.*;
import java.util.Arrays;

public class CurveFitTest {

    @Test
    public void testCurveFitting(){
        ImagePlus image = IJ.openImage("/Users/alexis/Anatomy_Project/TestSlices/SaturationTest/3.bmp");
        IntelligentScissors myScissors= new IntelligentScissors();
        myScissors.setImage(image);
        Point[] path = new  Point[2];

        Point one = new Point(130,700);
        Point two = new Point(350,700);

        path[0] = one;
        path[1] = two;

        PolygonRoi line = (PolygonRoi) myScissors.drawShortestPath(path);

        image.setRoi(line);
        image = image.flatten();
        IJ.save(image, "/Users/alexis/Anatomy_Project/TestSlices/SaturationTest/Test.bmp");
        IJ.save(myScissors.imageCost,"/Users/alexis/Anatomy_Project/TestSlices/SaturationTest/Cost.bmp");
        System.out.println("Roi line : \n" + line);

        Point[] linePoints = line.getContainedPoints();

        double[] xData = new double[path.length];
        double[] yData = new double[path.length];

        //Go through all the points and do interpolation for three points
        //Start one from the start, end one from the end
        //Only need to do this for the selected points?
        for (int i = 0; i < path.length ; i++) {
            xData[i] = path[i].x;
            yData[i] = path[i].y;
        }

        System.out.println("xData : " + Arrays.toString(xData));
        System.out.println("yData : " + Arrays.toString(yData));

        CurveFitter curveFitter = new CurveFitter(xData,yData);

        double perpendicularSlope;

        if(xData.length < 3) {
            curveFitter.doFit(CurveFitter.STRAIGHT_LINE);
            double[] params = curveFitter.getParams();
            if(params[1] == 0){
                perpendicularSlope = 10000000; //Large number.

            }else{
                perpendicularSlope = -1/params[1];
                System.out.println("Perpendicular Slope : " + perpendicularSlope);
            }

        }else {

            curveFitter.doFit(CurveFitter.POLY2);
            double[] params = curveFitter.getParams();

            double slope = params[1] + 2 * params[2] * xData[1];
            System.out.println("Poly slope : " + slope);

            //Start here

        }

    }

        /*for (int i = 1; i < linePoints.length-1 ; i++){
            xData[0] = (double) linePoints[i-1].x;
            xData[1] = (double) linePoints[i].x;
            xData[2] = (double) linePoints[i+1].x;
            yData[0] = (double) linePoints[i-1].y;
            yData[1] = (double) linePoints[i].y;
            yData[2] = (double) linePoints[i+1].y;

            System.out.println("xData : " + Arrays.toString(xData));
            System.out.println("yData : " + Arrays.toString(yData));

            CurveFitter curveFitter = new CurveFitter(xData,yData);
            curveFitter.doFit(CurveFitter.POLY2);
            double[] params = curveFitter.getParams();

            System.out.println(" Curve Parameters : " + Arrays.toString(params));
            System.out.println(curveFitter.getFormula());

        }*/
}
