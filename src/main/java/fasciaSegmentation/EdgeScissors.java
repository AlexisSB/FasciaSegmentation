package fasciaSegmentation;

import ij.ImagePlus;
import ij.gui.PolygonRoi;
import ij.gui.Roi;
import ij.measure.CurveFitter;
import ij.process.ImageConverter;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.PriorityQueue;

public class EdgeScissors extends IntelligentScissors {

    private ImagePlus edgeCosts;
    private Point[] middlePath;
    private Point[] edgePath;
    private Point[] edgeOppositePath;
    private Point[] userSelectedPoints;
    private Point[] edgeUserSelectedAnchorPoints;
    private Point[] oppositeEdgeUserSelectedAnchorPoints;
    private boolean edgeToggle;


    public EdgeScissors(){
        super();
        System.out.println("Creating Edge Scissors");
    }

    public void setMiddlePath(final Point[] path){
        this.middlePath = path;
    }
    public void setUserSelectedPoints(final Point[] containedPoints) {
        this.userSelectedPoints = containedPoints;
    }

    /**
     * Resets all the path costs and directions in the node array.
     */
    @Override
    public void reset(){
        setupInfoMatrix();
        middlePath = null;
        edgePath = null;
        edgeOppositePath = null;
        userSelectedPoints = null;
        edgeUserSelectedAnchorPoints = null;
        oppositeEdgeUserSelectedAnchorPoints = null;
    }

    public Roi getMiddlePath(){
        System.out.println("Getting the middle line roi");
        assert( userSelectedPoints != null);
        System.out.println("User Selected Points : " + Arrays.toString(userSelectedPoints));
        edgeToggle = false;
        PolygonRoi pathRoi = (PolygonRoi) drawShortestPath(userSelectedPoints);
        middlePath = pathRoi.getContainedPoints();
        edgeToggle = true;

        System.out.println("User Selected Points : " + Arrays.toString(userSelectedPoints));
        System.out.println("path Points : " + Arrays.toString(pathRoi.getContainedPoints()));

        return pathRoi;
    }

    public Roi getEdgeRoi(){
        System.out.println("Getting the edge roi");
        //Second approach uses the first and last normal points as nodes for the edge line.
        getEdge();
        Point start = edgePath[1];
        Point end = edgePath[edgePath.length-2];

        Point[] userSelectedPoint = new Point[2];
        userSelectedPoint[0] = start;
        userSelectedPoint[1] = end;
        PolygonRoi pathRoi = (PolygonRoi) drawShortestPath(userSelectedPoint);
        return pathRoi;
    }

    public Roi getEdgeComplementRoi(){
        System.out.println("Getting the opposite edge roi");
        getEdge();
        Point start = edgeOppositePath[1];
        Point end = edgeOppositePath[edgeOppositePath.length-2];
        Point[] userSelectedPoint = new Point[2];
        userSelectedPoint[0] = start;
        userSelectedPoint[1] = end;
        PolygonRoi pathRoi = (PolygonRoi) drawShortestPath(userSelectedPoint);
        return pathRoi;
    }

    public void getEdge(){
        double[] xData = new double[3];
        double[] yData = new double[3];

        edgePath = new Point[middlePath.length];
        edgeOppositePath = new Point[middlePath.length];

        for (int i = 0 ; i < middlePath.length; i++){
            edgePath[i] = new Point (middlePath[i].x, middlePath[i].y);
            edgeOppositePath[i] = new Point(middlePath[i].x, middlePath[i].y);
        }

        //For each point on the path
        for (int i = 1 ; i < middlePath.length-1; i++){
            //Set up xData and yData
            xData[0] = middlePath[i-1].x;
            xData[1] = middlePath[i].x;
            xData[2] = middlePath[i+1].x;
            yData[0] = middlePath[i-1].y;
            yData[1] = middlePath[i].y;
            yData[2] = middlePath[i+1].y;
            //get the slope of the normal line
            double slope = getPerpendicularSlopeHelper(xData,yData);
            //get a point to move to
            Point direction = getQuadrantDirection(slope);
            //calculate the resulting point.
            //edgePath[i].x += 5 * direction.x;
            //edgePath[i].y += 5 * direction.y;

            edgePath[i] = movePointToEdge(edgePath[i], direction);

            //Get opposite direction
            Point oppositeDirection = new Point((-1*direction.x), (-1*direction.y));

            edgeOppositePath[i] = movePointToEdge(edgeOppositePath[i], oppositeDirection);

        }

    }

    public Roi getEdgeRoiWithAnchors(){
        System.out.println("Getting the edge roi with anchors");
        getEdgeWithAnchors();
        PolygonRoi pathRoi = (PolygonRoi) drawShortestPath(edgeUserSelectedAnchorPoints);
        return pathRoi;
    }

    public Roi getOppositeEdgeRoiWithAnchors(){
        System.out.println("Getting the opposite edge roi using the user selected anchors");
        getEdgeWithAnchors();
        PolygonRoi pathRoi = (PolygonRoi) drawShortestPath(oppositeEdgeUserSelectedAnchorPoints);
        return pathRoi;
    }

    public void getEdgeWithAnchors(){
        System.out.println("getEdge helper start");
        double[] xData = new double[3];
        double[] yData = new double[3];
        edgeUserSelectedAnchorPoints = new Point[userSelectedPoints.length];
        oppositeEdgeUserSelectedAnchorPoints = new Point[userSelectedPoints.length];

        edgePath = new Point[middlePath.length];
        edgeOppositePath = new Point[middlePath.length];

        for (int i = 0 ; i < middlePath.length; i++){
            edgePath[i] = new Point (middlePath[i].x, middlePath[i].y);
            edgeOppositePath[i] = new Point(middlePath[i].x, middlePath[i].y);
        }

        //For each user selected point, find it in the middlePath array.
        int lastIndex = 0;
        for(int i = 0 ; i < userSelectedPoints.length; i++){
            System.out.println("Search of points started");
            System.out.println("User Selected Points : " + Arrays.toString(userSelectedPoints));
            System.out.println("Current Points : " + userSelectedPoints[i]);
            System.out.println("Middle Points : " +  Arrays.toString(middlePath));
            //FIXME
            //Search for the point that matches
            int index = -1;
            for( int j = lastIndex ; j < middlePath.length; j++){
                if(middlePath[j].x == userSelectedPoints[i].x && middlePath[j].y == userSelectedPoints[i].y){
                    index = j;
                    lastIndex = index;
                    break;
                }
            }
            assert(index != -1);
            if( index == 0 ){
                index++;
            }
            if( index == middlePath.length-1){
                index--;
            }
            System.out.println("Found at index : " + index);
            xData[0] = middlePath[index-1].x;
            xData[1] = middlePath[index].x;
            xData[2] = middlePath[index+1].x;
            yData[0] = middlePath[index-1].y;
            yData[1] = middlePath[index].y;
            yData[2] = middlePath[index+1].y;

            //Interpolate and calculate the normal points
            double slope = getPerpendicularSlopeHelper(xData,yData);
            Point direction = getQuadrantDirection(slope);
            edgeUserSelectedAnchorPoints[i] = movePointToEdge(edgePath[index], direction);

            Point oppositeDirection = new Point((-1*direction.x), (-1*direction.y));
            oppositeEdgeUserSelectedAnchorPoints[i] = movePointToEdge(edgeOppositePath[index], oppositeDirection);

        }
        System.out.println("getEdge helper end");
    }

    private Point movePointToEdge(Point start, Point direction){
        //Use greedy active contour movement.
        double pointCost = 255 - edgeCosts.getPixel(start.x, start.y)[0];

        Point newPoint = new Point(start.x + direction.x, start.y + direction.y);

        double newCost = 255 - edgeCosts.getPixel(newPoint.x, newPoint.y)[0];
        while(newCost < pointCost ){
            pointCost = newCost;
            newPoint = new Point(newPoint.x + direction.x, newPoint.y + direction.y);
            newCost = 255 - edgeCosts.getPixel(newPoint.x, newPoint.y)[0];
        }

        return newPoint;
    }

    /**
     * Approximates the normal to a function.
     * Uses the imageJ curvefitter tool to do quadratic interpolation from three points.
     * Quadratic equation then used to calculate slope of tangent line and normal line.
     * The data passed to this function comes from the interpolated line between selected points.
     * @param xData - x coordinates of three points
     * @param yData - y coordinates of three points.
     * @return the approximate slope of the normal line.
     */
    private double getPerpendicularSlopeHelper(double[] xData, double[] yData){

        CurveFitter curveFitter = new CurveFitter(xData,yData);

        double perpendicularSlope;
        double[] params;
        if(xData.length < 3) {
            curveFitter.doFit(CurveFitter.STRAIGHT_LINE);
            params = curveFitter.getParams();
            //System.out.println("Parameters: " + Arrays.toString(params));
            if(params[1] == 0){
                perpendicularSlope = 10000000; //Large number.

            }else{
                perpendicularSlope = -1/params[1];
                System.out.println("Perpendicular Slope : " + perpendicularSlope);
            }

        }else { //Multi point do a quadratic interpolation

            curveFitter.doFit(CurveFitter.POLY2);
            params = curveFitter.getParams();

            //System.out.println("Parameters : " + Arrays.toString(params));
            double slope = params[1] + 2 * params[2] * xData[1];
            assert(slope != 0);
            //System.out.println("Poly slope : " + slope);

            //Start here
            perpendicularSlope = -1/slope;


        }

        return perpendicularSlope;

    }

    /**
     * Returns the approximate direction to move in for a given slope of line.
     * Break up space into 3x3 pixel grid and map possible slopes onto it.
     * Provides normalisation for large slopes and meaningful direction.
     * @param slope
     * @return vector in form of point approximating the slope direction.
     */
    public Point getQuadrantDirection(double slope){

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

    /**
     *  Converts RGB image into imageCost channel image used for scissors.
     *  Converts the imageCost to an edges map for the edge scissors.
     * @param src - RGB image of slice
     */
    @Override
    public void setImage(ImagePlus src){
        ImagePlus temp = (ImagePlus) src.clone();
        ImageConverter ic = new ImageConverter(temp);
        ic.convertToHSB();
        temp.setSlice(2);
        ImagePlus saturation = new ImagePlus();
        saturation.setProcessor(temp.getProcessor());
        this.saturation = saturation;
        this.edgeCosts = getSobel(saturation, 5);

        setupInfoMatrix();
    }

   /* *//**
     * Creates a shortest path line between series of points of interest.
     * Generates a polygon roi to represent the path.
     * Using shortest path search point to point for each point in array.
     * @param
     * @return polygon roi representing shortest path.
     *//*
    @Override
    public Roi drawShortestPath(Point[] selectedPoints){
        ArrayList<Point> path = new ArrayList<Point>();

        //Reset the nodes so previous search paths dont interfer.
        setupInfoMatrix();

        if (selectedPoints.length <2){
            for( int i =0; i < selectedPoints.length; i++){
                path.add(new Point(selectedPoints[i].x, selectedPoints[i].y));
            }
        }else {
            for (int i = 0; i < selectedPoints.length - 1; i++) {
                System.out.println("Start : " + selectedPoints[i] + " End : " + selectedPoints[i+1]);
                Point start = new Point(selectedPoints[i].x, selectedPoints[i].y);
                Point end = new Point(selectedPoints[i+1].x, selectedPoints[i+1].y);
                shortestPathSearch(start,end);
                ArrayList<Point> shortestPath = getShortestPath(start, end);
                System.out.println("Shortest path length : " + shortestPath.size());
                //Reverse to get points in correct order before adding to path.
                Collections.reverse(shortestPath);
                path.addAll(shortestPath);
            }
            //Add the last point.
            //path.add(selectedPoints[selectedPoints.length-1]);
        }
        System.out.println("\nPath size : " + path.size());
        System.out.println( "Path : " + path + "\n");
        //Convert Arraylist to x and y array for constructing the polygon roi.
        int[] xArray = new int[path.size()];
        int[] yArray = new int[path.size()];
        for (int i = 0; i < path.size(); i++) {
            xArray[i] = path.get(i).x;
            yArray[i] = path.get(i).y;
        }

        System.out.println("xArray : " + Arrays.toString(xArray));
        System.out.println("yArray : " + Arrays.toString(yArray)+ "\n");
        PolygonRoi pathRoi = new PolygonRoi(xArray, yArray, path.size(), Roi.POLYLINE);
        System.out.println("Path roi length : " + pathRoi.getContainedPoints().length);
        System.out.println("Path roi : " + Arrays.toString(pathRoi.getContainedPoints()));
        System.out.println();
        return pathRoi;

    }*/

    /* Calculates the path cost between two nodes for the shortest path search.
     * Requires knowledge of goal for distance heuristic.
     *
     * @param one - current point
     * @param two - next point on path
     * @param goal - end point in path search
     * @return the cost of moving from one to two.
     */
    @Override
    public double calculateLinkCost(Point one, Point two, Point goal) {

        //Apply extra cost to moving diagonal.
        int diagonalTest = Math.abs(two.x-one.x)+ Math.abs(two.y - one.y);
        double diagonalMultiplier = 1.0;
        if (diagonalTest == 2){
            diagonalMultiplier = 1.41;
        }

        int distanceFromEnd = Math.abs(goal.y-two.y)+Math.abs(goal.x-goal.x);
        int grayIntensity;
        if(edgeToggle) {
            grayIntensity = edgeCosts.getPixel(two.x, two.y)[0];
        }else{
            grayIntensity = saturation.getPixel(two.x, two.y)[0];
        }

        return ((255-grayIntensity) + distanceFromEnd)*diagonalMultiplier;

    }

}
