package fasciaSegmentation;

import ij.ImagePlus;
import ij.gui.PolygonRoi;
import ij.gui.Roi;
import ij.gui.StackWindow;
import ij.measure.CurveFitter;
import ij.plugin.filter.Convolver;
import ij.plugin.filter.GaussianBlur;
import ij.process.FloatProcessor;
import ij.process.ImageConverter;
import ij.process.ImageProcessor;
import org.omg.PortableInterceptor.SYSTEM_EXCEPTION;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.*;
import java.util.List;

public class PathFinder {

    /** Points used to move between centre and neighbours*/
    Point[] neighbours = new Point[9];
    ArrayList<Point> userSelectedPoints;
    ImagePlus centreLineCostMatrix;
    ImagePlus edgeCostMatrix;
    PixelNode[][] pathSearchMatrix;

    ArrayList<Point> centreLinePath;
    ArrayList<Point> edgeLinePath;
    ArrayList<Point> oppositeEdgeLinePath;

    boolean centreLineActive = false;
    boolean edgeLineActive = false;

    public PathFinder(){
        userSelectedPoints = new ArrayList<Point>();
        centreLinePath = new ArrayList<Point>();
        edgeLinePath = new ArrayList<Point>();
        oppositeEdgeLinePath = new ArrayList<Point>();

        //Set up the neighbours array
        int count = 0;
        for( int i = -1; i <= 1; i++){
            for(int j = -1; j <=1 ; j++){
                neighbours[count] = new Point(i,j);
                count++;
            }
        }

    }

    /**
     * Takes the current image and applies appropriate filters to get cost matrix for scissor functions.
     * Centre line cost is dependent on the saturation of the image.
     * Edge costing depends on the difference of gaussian and sobel filters.
     */
    public void setImage(ImagePlus src){
        //Create working copy of image
        ImagePlus temp = src.duplicate();

        //Convert to Saturation Image
        ImageConverter ic = new ImageConverter(temp);
        ic.convertToHSB();
        temp.setSlice(2);
        centreLineCostMatrix = new ImagePlus();
        centreLineCostMatrix.setProcessor(temp.getProcessor());

        //StackWindow lineCost = new StackWindow(centreLineCostMatrix);
        //lineCost.pack();

        //Calculate Edge Matrix
        //edgeCostMatrix = getSobel(temp, 5);
        //StackWindow edgeCost = new StackWindow(edgeCostMatrix);
        //edgeCost.pack();

        //Insert Dog Here
        //Testing DoG generation
        ic.convertToGray8();
        ImagePlus dog = getDoG(temp, 1,100);
//        StackWindow dogWindow = new StackWindow(dog);
//        dogWindow.pack();

        ImagePlus dogEdge = getSobel(dog, 5);
//        StackWindow dogEdgeWindow = new StackWindow(dogEdge);
//        dogEdgeWindow.setName("Dog Edges");
//        dogEdgeWindow.pack();

        edgeCostMatrix = dogEdge;

    }

    /**
     * Computes difference of gaussian for the given image.
     * Output is signed 32 bit image of the DoG.
     * @param originalImage
     * @param sigma1
     * @param sigma2
     * @return a 32 bit float image of the DoG.
     */
    public ImagePlus getDoG(final ImagePlus originalImage, final double sigma1, final double sigma2) {

        final int width = originalImage.getWidth();
        final int height = originalImage.getHeight();

        GaussianBlur gs = new GaussianBlur();

        ImageProcessor ip_1 = originalImage.getProcessor().duplicate();
        //gs.blur(ip_1, sigma1);
        gs.blurGaussian(ip_1, 0.4 * sigma1, 0.4 * sigma1, 0.0002);
        ImageProcessor ip_2 = originalImage.getProcessor().duplicate();
        //gs.blur(ip_2, sigma2);
        gs.blurGaussian(ip_2, 0.4 * sigma2, 0.4 * sigma2, 0.0002);

        ImageProcessor ip = new FloatProcessor(width, height);

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                float v1 = ip_1.getf(x, y);
                float v2 = ip_2.getf(x, y);
                ip.setf(x, y, v2 - v1);
            }
        }

        ImagePlus result = new ImagePlus("DoG", ip);
        return result;
    }

    /**
     * Setter for the currently selected roi points.
     * When called this will calculate the middle and edge paths ready for them to be called.
     * @param userSelectedPoints
     */
    public void setUserSelectedPoints(final Point[] userSelectedPoints) {
        this.reset();
        this.userSelectedPoints = new ArrayList<Point>();
        for( Point p : userSelectedPoints){
            this.userSelectedPoints.add(p);
        }

        findCentreLine();
        System.out.println("\nDone Centre Line");
        System.out.println("Centre Path : " + centreLinePath );
        System.out.println();

        findEdgeLines();
    }

    /**
     * Initialises the path search matrix and resets the search nodes.
     * Precondition : centreLine and edgeCost matrices must be set.
     */
    public void setUpPathSearchMatrix() {

        //y = height, x = width
        pathSearchMatrix = new PixelNode[centreLineCostMatrix.getHeight()][centreLineCostMatrix.getWidth()];
        System.out.println("Resetting the path search node matrix");
        System.out.println(pathSearchMatrix.length);
        System.out.println(pathSearchMatrix[0].length);
        System.out.println();
        for(int y = 0 ; y < pathSearchMatrix.length; y++){
            for(int x = 0 ; x < pathSearchMatrix[y].length; x++){
                pathSearchMatrix[y][x] = new PixelNode(x, y);
            }
        }

    }

    public Roi getCentreLinePathRoi() {
        //lineSmoothing(centreLinePath);
        int[] xArray = new int[centreLinePath.size()];
        int[] yArray = new int[centreLinePath.size()];
        for (int i = 0; i < centreLinePath.size(); i++) {
            xArray[i] = centreLinePath.get(i).x;
            yArray[i] = centreLinePath.get(i).y;
        }
        PolygonRoi pathRoi = new PolygonRoi(xArray, yArray, centreLinePath.size(), Roi.POLYLINE);
        System.out.println("User Selected Points : " + userSelectedPoints);
        return pathRoi;
    }

    public Roi getEdgeLinePathRoi() {
        System.out.println(edgeLinePath.size());
        int[] xArray = new int[edgeLinePath.size()];
        int[] yArray = new int[edgeLinePath.size()];
        for (int i = 0; i < edgeLinePath.size(); i++) {
            xArray[i] = edgeLinePath.get(i).x;
            yArray[i] = edgeLinePath.get(i).y;
        }

        PolygonRoi pathRoi = new PolygonRoi(xArray, yArray, edgeLinePath.size(), Roi.POLYLINE);
        System.out.println(" Edge : " + edgeLinePath);

        return pathRoi;
    }

    public Roi getOppositeEdgeLinePathRoi() {
        System.out.println(oppositeEdgeLinePath.size());
        int[] xArray = new int[oppositeEdgeLinePath.size()];
        int[] yArray = new int[oppositeEdgeLinePath.size()];
        for (int i = 0; i < oppositeEdgeLinePath.size(); i++) {
            xArray[i] = oppositeEdgeLinePath.get(i).x;
            yArray[i] = oppositeEdgeLinePath.get(i).y;
        }

        PolygonRoi pathRoi = new PolygonRoi(xArray, yArray, oppositeEdgeLinePath.size(), Roi.POLYLINE);
        System.out.println("Opposite Edge : " + oppositeEdgeLinePath);


        return pathRoi;
    }

    public Roi getEdgeBoxRoi(){

        System.out.println("Edge Line path size : " + edgeLinePath.size());
        System.out.println("Opposite Edge Line path size : " + oppositeEdgeLinePath.size());

        ArrayList<Point> path = new ArrayList<Point>();

        path.addAll(edgeLinePath);

        for(int i = oppositeEdgeLinePath.size()-1; i >=0  ; i--){
            path.add(oppositeEdgeLinePath.get(i));
        }
        System.out.println(path.size());

        int[] xArray = new int[path.size()];
        int[] yArray = new int[path.size()];

        for (int i = 0; i < path.size(); i++) {
            xArray[i] = path.get(i).x;
            yArray[i] = path.get(i).y;
        }

        PolygonRoi boxRoi = new PolygonRoi( xArray, yArray, edgeLinePath.size()+oppositeEdgeLinePath.size(),Roi.POLYGON );

        return boxRoi;
    }

    public boolean isCentreLineActive() {
        return centreLineActive;
    }

    public void setCentreLineActive(boolean centreLineActive) {
        this.centreLineActive = centreLineActive;
    }

    public boolean isEdgeLineActive() {
        return edgeLineActive;
    }

    public void setEdgeLineActive(boolean edgeLineActive) {
        this.edgeLineActive = edgeLineActive;
    }

    private void findCentreLine(){

        ArrayList<Point> path = findShortestPath(userSelectedPoints, centreLineCostMatrix);
        centreLinePath.addAll(path);

    }

    private void findEdgeLines(){

        //Edge lines based on normal lines
        ArrayList<ArrayList<Point>> allEdgePoints = findEdgeUsingNormalPoints();
        edgeLinePath.addAll(allEdgePoints.get(0));
        oppositeEdgeLinePath.addAll(allEdgePoints.get(1));

        System.out.println("\nStarting Smoothing");
        edgeLinePath = (ArrayList<Point>) lineSmoothing(edgeLinePath);
        oppositeEdgeLinePath = (ArrayList<Point>) lineSmoothing(oppositeEdgeLinePath);

/*
        //Edge lines based on shortest path
        ArrayList<ArrayList<Point>> edgePoint = findEdgeAnchorPoints();
        ArrayList<Point> edgeAnchorPoints = edgePoint.get(0);
        ArrayList<Point> oppositeEdgeAnchorPoints = edgePoint.get(1);

        System.out.println("Edge Anchors : " + edgeAnchorPoints);
        System.out.println("Opposite Edge Anchors : " + oppositeEdgeAnchorPoints);
        System.out.println();

        ArrayList<Point> path = findShortestPath(edgeAnchorPoints, edgeCostMatrix);
        edgeLinePath.addAll(path);
        System.out.println();

        ArrayList<Point> oppositePath = findShortestPath(oppositeEdgeAnchorPoints, edgeCostMatrix);
        oppositeEdgeLinePath.addAll(oppositePath);
*/

    }

    /*private ArrayList<ArrayList<Point>> findEdgeAnchorPoints() {

        ArrayList<Point> edgeAnchorPoints = new ArrayList<Point>();
        ArrayList<Point> oppositeEdgeAnchorPoints = new ArrayList<Point>();

        assert(centreLinePath.size() > 2);

        //For each point that the user has selected
        for(Point p : userSelectedPoints){
            //Find that point in the middle path
            int pointIndex = centreLinePath.indexOf(p);
            if(pointIndex ==-1){
                System.err.println("Point not found in middle line");
                break;
            }
            //Select the points next to it
            //Check if first and last points.
            // Do quadratic interpolation of those points
            //get the normal direction
            double slope;
            System.out.println("Location : " + pointIndex + " Size : " +centreLinePath.size());

            if( pointIndex != 0 && pointIndex != centreLinePath.size()-1){ // If point in between start and end of line
                slope = findNormalDirection(centreLinePath.subList(pointIndex-1, pointIndex+2));
            }else if(pointIndex == 0){ //If point at the start of line
                slope = findNormalDirection(centreLinePath.subList(0,3));
            }else { // if the point at the end of the line
                assert (pointIndex == centreLinePath.size()-1);
                slope = findNormalDirection(centreLinePath.subList(pointIndex-2, pointIndex+1));
            }

            //convert it to a quadrant direction.
            Point direction = convertSlopeToDiscreteDirection(slope);
            //Move the point along that direction until it reaches a minimum value.
            Point newEdgePoint = new Point(p.x,p.y);
            newEdgePoint = movePointToEdge(newEdgePoint, direction, 10);
            //Add the point to the edge anchors list
            edgeAnchorPoints.add(newEdgePoint);

            Point newOppositeEdgePoint = new Point(p.x,p.y);
            Point oppositeDirection = new Point((-1*direction.x), (-1*direction.y));
            newOppositeEdgePoint = movePointToEdge(newOppositeEdgePoint, oppositeDirection, 10);
            //Add the point to the edge anchors list
            oppositeEdgeAnchorPoints.add(newOppositeEdgePoint);

        }

        ArrayList<ArrayList<Point>> output = new ArrayList<ArrayList<Point>>();
        output.add(edgeAnchorPoints);
        output.add(oppositeEdgeAnchorPoints);

        System.out.println("Edge Anchor Function output : " + output);
        System.out.println();

        return output;
    }*/


    public int getMaxThickness() {
        return maxThickness;
    }

    public void setMaxThickness(int maxThickness) {
        this.maxThickness = maxThickness;
    }

    private int maxThickness = 10;

    private ArrayList<ArrayList<Point>> findEdgeUsingNormalPoints() {

        ArrayList<Point> edgePoints = new ArrayList<Point>();
        ArrayList<Point> oppositeEdgePoints = new ArrayList<Point>();

        assert(centreLinePath.size() > 2);
        Point previousEdge = null;
        Point previousOppositeEdge = null;
        //For each point that the user has selected
        for(int pointIndex = 1 ; pointIndex < centreLinePath.size()-1; pointIndex++){

            //Find that point in the middle path
            //Select the points next to it
            //Check if first and last points.
            //Do quadratic interpolation of those points
            //get the normal direction

            double slope;
            //System.out.println("Location : " + pointIndex + " Size : " + centreLinePath.size());

            // If point in between start and end of line
            slope = findNormalDirection(centreLinePath.subList(pointIndex - 1, pointIndex + 2));
            System.out.println("Normal Slope : " + slope);
            //convert it to a quadrant direction.
            Point direction = convertSlopeToDiscreteDirection(slope);
            System.out.println("Direction : " + direction);
            System.out.println(" Mid point : " + centreLinePath.get(pointIndex));

            //Move the point along that direction until it reaches a minimum value.
            Point newEdgePoint = new Point(centreLinePath.get(pointIndex).x, centreLinePath.get(pointIndex).y);

            newEdgePoint = movePointToEdge(newEdgePoint, direction, previousEdge, maxThickness);
            previousEdge = new Point(newEdgePoint.x, newEdgePoint.y);
            System.out.println(" New Edge Point : " + newEdgePoint);
            //Add the point to the edge anchors list
            edgePoints.add(newEdgePoint);

            Point newOppositeEdgePoint = new Point(centreLinePath.get(pointIndex).x, centreLinePath.get(pointIndex).y);
            Point oppositeDirection = new Point((-1 * direction.x), (-1 * direction.y));
            System.out.println("Opposite Direction : " + oppositeDirection);
            newOppositeEdgePoint = movePointToEdge(newOppositeEdgePoint, oppositeDirection, previousOppositeEdge,  maxThickness);
            previousOppositeEdge = new Point(newOppositeEdgePoint.x, newOppositeEdgePoint.y);
            System.out.println(" Opposite Edge Point : " + newOppositeEdgePoint + "\n");
            //Add the point to the edge anchors list
            oppositeEdgePoints.add(newOppositeEdgePoint);

            /*
            }else if(pointIndex == 0){ //If point at the start of line
                continue;
                //slope = findNormalDirection(centreLinePath.subList(0,3));
            }else { // if the point at the end of the line
                continue;
                //assert (pointIndex == centreLinePath.size()-1);
                //slope = findNormalDirection(centreLinePath.subList(pointIndex-2, pointIndex+1));
            }
*/


        }

        ArrayList<ArrayList<Point>> output = new ArrayList<ArrayList<Point>>();
        output.add(edgePoints);
        output.add(oppositeEdgePoints);

        System.out.println("Edge Anchor Function output : " + output);
        System.out.println();

        return output;
    }

    public double getDistanceCostMultiplier() {
        return distanceCostMultiplier;
    }

    public void setDistanceCostMultiplier(double distanceCostMultiplier) {
        this.distanceCostMultiplier = distanceCostMultiplier;
    }

    private double distanceCostMultiplier = 1.0;



    private Point movePointToEdge(Point start, Point direction, Point previous,  int maxThickness){

        //Scan ahead the max width number of pixels and pick the point with the lowest cost
        double oldCost = 255 - edgeCostMatrix.getPixel(start.x, start.y)[0];

        Point newPoint = new Point(start.x + direction.x, start.y + direction.y);
        int distanceCost = 0;
        if(previous != null) {
            distanceCost = Math.abs(newPoint.x - previous.x) + Math.abs(newPoint.y-previous.y);
        }
        double newCost = (255 - edgeCostMatrix.getPixel(newPoint.x, newPoint.y)[0]) + distanceCost;
        Point minPoint = new Point(newPoint.x, newPoint.y);
        for (int i = 0; i < maxThickness; i++ ){
            newPoint = new Point(newPoint.x + direction.x, newPoint.y + direction.y);
            if(previous != null) {
                distanceCost = Math.abs(newPoint.x - previous.x) + Math.abs(newPoint.y-previous.y);
                distanceCost *= distanceCostMultiplier;
            }
            newCost = (255 - edgeCostMatrix.getPixel(newPoint.x, newPoint.y)[0]) + distanceCost;

            if(newCost <= oldCost){
                oldCost = newCost;
                minPoint = new Point(newPoint.x, newPoint.y);
            }
        }

        return minPoint;
        /*//Use greedy active contour movement.
        double pointCost = 255 - edgeCostMatrix.getPixel(start.x, start.y)[0];

        Point newPoint = new Point(start.x + direction.x, start.y + direction.y);

        double newCost = 255 - edgeCostMatrix.getPixel(newPoint.x, newPoint.y)[0];
        System.out.println("oldCost : " + pointCost);
        System.out.println("newCost : " + newCost);


        while(newCost <= pointCost ){
            pointCost = newCost;
            newPoint = new Point(newPoint.x + direction.x, newPoint.y + direction.y);
            newCost = 255 - edgeCostMatrix.getPixel(newPoint.x, newPoint.y)[0];
            System.out.println("newCost : " + newCost);
        }

        return newPoint;*/
    }



    private Point convertSlopeToDiscreteDirection(double slope) {
        double angle = Math.atan(slope);
        angle = Math.toDegrees(angle);
        //System.out.println("Angle : " + angle);

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


    public int getSmoothingThreshold() {
        return smoothingThreshold;
    }

    private int smoothingThreshold = 30;
    public void setSmoothingThreshold(int smoothingThreshold) {
        this.smoothingThreshold = smoothingThreshold;
    }

    public int getPointStep() {
        return pointStep;
    }

    public void setPointStep(int pointStep) {
        this.pointStep = pointStep;
    }

    private int pointStep = 4;

    protected List<Point> lineSmoothing(ArrayList<Point> edgePath){

        //Approach 1
        final int deltaXThreshold = 5;
        final int deltaYThreshold = 5;
        final int numberOfPointsToIncludeInMean = 10;

        for(int i = 0; i < edgePath.size()-1; i++){
            Point one = edgePath.get(i);
            Point two = edgePath.get(i+1);
            int deltaX = Math.abs(two.x - one.x);
            int deltaY = Math.abs(two.y - one.y);

            if(deltaX > deltaXThreshold || deltaY > deltaYThreshold){
                System.out.println("Smoothing points : " + one + " and " + two);
                //Check if point too close to ends of line
                if(i > edgePath.size()-(numberOfPointsToIncludeInMean/2)){
                    //Pick the last ten points
                    //calculate mean
                    int sumX = 0;
                    int sumY = 0;
                    for( int j = numberOfPointsToIncludeInMean; j > 0 ; j--){
                        int size = edgePath.size();
                        Point first = edgePath.get(size-j -2);
                        Point second = edgePath.get(size-j-1);
                        sumX += Math.abs(two.x - one.x);
                        sumY += Math.abs(two.y - one.y);
                    }
                    int averageDeltaX = sumX/numberOfPointsToIncludeInMean;
                    int averageDeltaY = sumY/numberOfPointsToIncludeInMean;
                    Point newPoint = new Point(one.x + averageDeltaX, one.y + averageDeltaY);
                    two = newPoint;
                } else if(i < numberOfPointsToIncludeInMean){
                    //Pick the first ten points
                    //calculate mean
                    int sumX = 0;
                    int sumY = 0;
                    for( int j = 0 ; j < numberOfPointsToIncludeInMean ; j++){
                        Point first = edgePath.get(j);
                        Point second = edgePath.get(j+1);
                        sumX += Math.abs(two.x - one.x);
                        sumY += Math.abs(two.y - one.y);
                    }
                    int averageDeltaX = sumX/numberOfPointsToIncludeInMean;
                    int averageDeltaY = sumY/numberOfPointsToIncludeInMean;
                    Point newPoint = new Point(one.x + averageDeltaX, one.y + averageDeltaY);
                    two = newPoint;

                }else {
                    //calculate mean
                    int sumX = 0;
                    int sumY = 0;
                    for (int j = -numberOfPointsToIncludeInMean / 2; j < numberOfPointsToIncludeInMean / 2; j++) {
                        Point first = edgePath.get(i + j);
                        Point second = edgePath.get(i + j + 1);
                        sumX += Math.abs(two.x - one.x);
                        sumY += Math.abs(two.y - one.y);
                    }
                    int averageDeltaX = sumX / numberOfPointsToIncludeInMean;
                    int averageDeltaY = sumY / numberOfPointsToIncludeInMean;
                    Point newPoint = new Point(one.x + averageDeltaX, one.y + averageDeltaY);
                    two = newPoint;
                }

            }


        }

        //Approach 2
        //If edge cost of point different to average of last few points then replace with average step.
        /*for(int i = 0; i < edgePath.size()-1; i++) {
            Point one = edgePath.get(i);
            Point two = edgePath.get(i + 1);
            int deltaX = Math.abs(two.x - one.x);
            int deltaY = Math.abs(two.y - one.y);
            int costOne = edgeCostMatrix.getPixel(one.x, one.y)[0];
            int costTwo = edgeCostMatrix.getPixel(two.x, two.y)[0];
            int deltaCost = Math.abs(costTwo-costOne);
            if(deltaCost > smoothingThreshold || ){
                edgePath.remove(i+1);
                i--;
            }

        }*/

        //Approach 3
        //Check the angle every 4 points, if the angle to big then reject the point.

        System.out.println("Path Size : " + edgePath.size());

        /*for(int i = 0; i < edgePath.size()-pointStep; i++) {
            Point one = edgePath.get(i);
            Point two = edgePath.get(i + pointStep/2);
            Point three = edgePath.get(i + pointStep);
//            System.out.println("Point one : " +one);
//            System.out.println("Point two : " + two);
//            System.out.println("Point three : " + three);
            Point vectorOne = new Point(two.x-one.x, two.y-one.y);
//            System.out.println("Vector One : " + vectorOne);

            Point vectorTwo = new Point( three.x - two.x, three.y - two.y);
//            System.out.println("Vector two : " + vectorTwo);

            double lengthOne = Point2D.distance(one.x, one.y, two.x, two.y);
            double lengthTwo = Point2D.distance(two.x, two.y, three.x, three.y);
//            System.out.println("Length One : " + lengthOne + " Length Two : " + lengthTwo);

            double angle = ((vectorOne.x*vectorTwo.x)+(vectorOne.y*vectorTwo.y))/(lengthOne*lengthTwo);
            angle = Math.acos(angle);
            angle = Math.toDegrees(angle);
//            System.out.println("Angle : " + angle);

            if(angle > smoothingThreshold){
                //Delete Point
                edgePath.remove(i+pointStep);
                i--;
                *//*System.out.println("Angle : " + angle);
                System.out.print("Adjusting point : " + three);
                //Replace Point
                //for points before it || until is zero
                int sumX = 0;
                int sumY = 0;
                for(int j = 0 ;j < pointStep;j++){
                    Point p = edgePath.get(i+j);
                    Point q = edgePath.get(i+j+1);
                    sumX += (q.x-p.x);
                    sumY += (q.y-p.y);
                }
                int averageDeltaX = sumX / pointStep-1;
                int averageDeltaY = sumY / pointStep-1;
                //System.out.println("Deltas : " + averageDeltaX + " " + averageDeltaY);
                Point newPoint = new Point(two.x + averageDeltaX, two.y + averageDeltaY);
                edgePath.set(i+pointStep, newPoint);
                System.out.println(" ---> " + edgePath.get(i+pointStep));
                //Calculate average change in x and y
                //Apply to the last point*//*
            }*/

        //}
        System.out.println("Path Size : " + edgePath.size());
        //Keep track of the change in x and y between points
        //If its larger than some threshold move the point.


        //For each point in the edge line
        //if the point is larger than some distance from the previous point
        //Replace it with average of some sort
        //Try deleting first?

        return edgePath;
    }

    protected double findNormalDirection(List<Point> points) {
        //Current version allows only three points to determine function.
        //System.out.println("Interpolating points : " + points + "\n Size : " + points.size());

        assert(points.size() == 3);

        double[] xData = new double[points.size()];
        double[] yData = new double[points.size()];

        for( int i = 0; i < points.size(); i++ ){
            xData[i] = (double) points.get(i).x;
            yData[i] = (double) points.get(i).y;
        }

        CurveFitter curveFitter = new CurveFitter(xData,yData);

        double perpendicularSlope;
        double[] params;

        //Multi point do a quadratic interpolation
        curveFitter.doFit(CurveFitter.POLY2);
        params = curveFitter.getParams();

        double slope = params[1] + 2 * params[2] * xData[1];
        System.out.println("Tangent slope : " + slope);
        //Multiply slope by -1 because of different coordinate system in the picture to the curve fitter
        slope *= 1;
        System.out.println("Flipped slope : " + slope);
        if(slope == 0){
            return 100000000; // large number for vertical
        }else {
            perpendicularSlope = -1 / slope;
            return perpendicularSlope;
        }
    }

    private ArrayList<Point> findShortestPath(ArrayList<Point> points, ImagePlus costMatrix){
        ArrayList<Point> path = new ArrayList<Point>();

        if(points.size() < 2){
            for( int i =0; i < points.size(); i++){
                path.add(new Point(points.get(i).x, points.get(i).y));
            }
            return path;
        }

        //Points are the user selected points.
        for (int i = 0; i < points.size()- 1; i++) {

            Point start = new Point(points.get(i).x, points.get(i).y);
            Point end = new Point(points.get(i+1).x, points.get(i+1).y);;

            System.out.println("Start : " + start + " End : " + end);

            findShortestPathHelperSearch(start,end, costMatrix);

            ArrayList<Point> shortestPath = findShortestPathHelperTracePath(start, end);

            System.out.println(shortestPath);
            assert(shortestPath != null);
            path.addAll(shortestPath);
            path.remove(path.size()-1);
        }

        path.add(points.get(points.size()-1));
        return path;
    }

    private void findShortestPathHelperSearch(Point start, Point goal, ImagePlus costMatrix){
        PriorityQueue<PixelNode> pq = new PriorityQueue<PixelNode>();

        setUpPathSearchMatrix();
        //Setup first node
        PixelNode seed = pathSearchMatrix[start.y][start.x];
        seed.cost = 0;

        pq.add(seed);

        while(!pq.isEmpty()){
            PixelNode node = pq.poll();
            //IJ.log(node.getPoint() + "");

            if(node.getPoint().equals(goal)){
                return;
            }

            //Set point to visited;
            node.state = State.VISITED;

            //Explore the neighbours
            for( int i =0; i < neighbours.length; i++){
                Point neighbour = node.getPoint();
                neighbour.x += neighbours[i].x;
                neighbour.y += neighbours[i].y;

                //Bounds checking to make sure point inside the image
                if(neighbour.x > 0 && neighbour.y > 0 && neighbour.x < costMatrix.getWidth() && neighbour.y < costMatrix.getHeight()) {

                    PixelNode nextNode = pathSearchMatrix[neighbour.y][neighbour.x];

                    if (nextNode.state == State.INITIAL) {
                        //Check cost and update if less
                        double totalcost = node.cost + calculateLinkCost(node.getPoint(), nextNode.getPoint(), goal, costMatrix);
                        nextNode.cost = totalcost;
                        nextNode.previous = neighbours[8 - i];
                        nextNode.state = State.ACTIVE;
                        pq.add(nextNode);

                    } else if (nextNode.state == State.ACTIVE) {
                        //Do nothing
                        double totalcost = node.cost + calculateLinkCost(node.getPoint(), nextNode.getPoint(),goal, costMatrix);
                        if (totalcost < nextNode.cost) {
                            pq.remove(nextNode);
                            nextNode.cost = totalcost;
                            nextNode.previous = neighbours[8 - i];
                            nextNode.state = State.ACTIVE;
                            pq.add(nextNode);
                        }
                    }
                }
            }
        }
    }

    private ArrayList<Point> findShortestPathHelperTracePath(Point start, Point goal){
        Point temp = new Point(goal.x, goal.y);
        ArrayList<Point> path = new ArrayList<Point>();
        path.add(new Point(goal.x, goal.y));
        while (!temp.equals(start)) {
            Point previous = pathSearchMatrix[temp.y][temp.x].previous;
            if (previous != null) {
                temp.x += previous.x;
                temp.y += previous.y;
                path.add(new Point(temp.x, temp.y));
            }else{
                System.err.println("Shortest path between two points has not been explored");
                System.err.println("Point : " + temp);
                return null;
            }
        }

        Collections.reverse(path);
        return path;
    }

    private double calculateLinkCost(Point one, Point two, Point goal, ImagePlus costMatrix){
        //Apply extra cost to moving diagonal.
        int diagonalTest = Math.abs(two.x-one.x)+ Math.abs(two.y - one.y);
        double diagonalMultiplier = 1.0;
        if (diagonalTest == 2){
            diagonalMultiplier = 1.41;
        }


        int distanceFromEnd = Math.abs(goal.y-two.y)+Math.abs(goal.x-goal.x);
        int intensityCost = 255 - costMatrix.getPixel(two.x, two.y)[0];
        int angleCost = 180 - (int)calculateAngleCost(one,two);
        //double proximityCost = calculateProximityCost(two);
        //System.out.println("AngleCost : " + angleCost);
        return (intensityCost + distanceFromEnd + angleCost)*diagonalMultiplier;

    }

    protected double calculateProximityCost(Point one){

        if(centreLinePath == null || centreLinePath.isEmpty()){
            return 0;
        }else{
            Point closestPoint;
            int distance = 10000;
            for(Point p : centreLinePath){
                int tempDistance = Math.abs(one.x - p.x) + Math.abs(one.y - p.y);
                if( tempDistance < distance){
                    closestPoint = p;
                    distance = tempDistance;
                }
            }

            double cost = 4/ Math.pow(distance, 2);

            return cost;
        }

    }

    protected double calculateAngleCost(Point pointA, Point pointB){

        Point previous = pathSearchMatrix[pointA.y][pointA.x].previous;

        if(previous == null ){
            return 0;
        }else{
            return calculateAngleCost(previous, pointA,pointB);
        }

    }


    protected double calculateAngleCost(final Point one, final Point two, final Point three){
        double oneTwo;
        double twoThree;
        double oneThree;
        oneTwo = Point2D.distance(one.x, one.y, two.x, two.y);
        twoThree = Point2D.distance(two.x, two.y, three.x, three.y);
        oneThree = Point2D.distance(one.x, one.y, three.x, three.y);
        double angle = Math.acos((Math.pow(oneThree,2) -Math.pow(oneTwo,2) - Math.pow(twoThree, 2))/ (-2*(oneTwo)*(twoThree)));
        angle = Math.toDegrees(angle);
        return angle;
    }

    /**
     * Calculates horizontal and vertical Sobel filters for the input image.
     * Adapted from Weka Trainable Segmentation Feature Stack class.
     * @param originalImage
     * @param sigma
     * @return
     */
    private ImagePlus getSobel( final ImagePlus originalImage, final float sigma) {
        if (Thread.currentThread().isInterrupted())
            return null;

        final int width = originalImage.getWidth();
        final int height = originalImage.getHeight();

        GaussianBlur gs = new GaussianBlur();
        ImageProcessor ip_x = originalImage.getProcessor().duplicate().convertToFloat();
        //gs.blur(ip_x, sigma);
        gs.blurGaussian(ip_x, 0.4 * sigma, 0.4 * sigma, 0.0002);
        Convolver c = new Convolver();
        float[] sobelFilter_x = {1f, 2f, 1f, 0f, 0f, 0f, -1f, -2f, -1f};
        c.convolveFloat(ip_x, sobelFilter_x, 3, 3);

        ImageProcessor ip_y = originalImage.getProcessor().duplicate().convertToFloat();
        //gs.blur(ip_y, sigma);
        gs.blurGaussian(ip_y, 0.4 * sigma, 0.4 * sigma, 0.0002);
        c = new Convolver();
        float[] sobelFilter_y = {1f, 0f, -1f, 2f, 0f, -2f, 1f, 0f, -1f};
        c.convolveFloat(ip_y, sobelFilter_y, 3, 3);

        ImageProcessor ip = new FloatProcessor(width, height);

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                float s_x = ip_x.getf(x, y);
                float s_y = ip_y.getf(x, y);
                ip.setf(x, y, (float) Math.sqrt(s_x * s_x + s_y * s_y));
            }
        }

        ImagePlus result = new ImagePlus("Edges", ip);

        //Convert to 8 bit greyscale
        ImageConverter ic = new ImageConverter(result);
        ic.convertToGray8();

        return  result;
    }

    public void reset() {
        userSelectedPoints.clear();
        centreLinePath.clear();
        edgeLinePath.clear();
        oppositeEdgeLinePath.clear();
        setUpPathSearchMatrix();

    }

    /** Different states of nodes during search */
    public enum State {INITIAL, ACTIVE, VISITED};

    /** Inner class for the nodes used in the shortest path search.
     * @author Alexis Barltrop
     */
    protected class PixelNode implements Comparator<PixelNode>, Comparable<PixelNode> {

        int x;
        int y;
        double cost;
        Point previous;
        State state;

        public PixelNode(Point p){
            this.x = p.x;
            this.y = p.y;
            state = State.INITIAL;
            previous = null;
            this.cost = Double.MAX_VALUE;
        }

        public PixelNode(int x, int y){
            this.x = x;
            this.y = y;
            state = State.INITIAL;
            previous = null;
            this.cost = Double.MAX_VALUE;
        }

        @Override
        public int compareTo(PixelNode o) {
            double output = this.cost-o.cost;
            return (int)output;
        }

        @Override
        public int compare(PixelNode o1, PixelNode o2) {
            double output = o1.cost - o2.cost;
            return (int)output;
        }

        Point getPoint(){
            return new Point(x,y);
        }

    }


}
