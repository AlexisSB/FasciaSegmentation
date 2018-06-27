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

    private Point[] middlePath;
    private Point[] edgePath;
    private Point[] edgeOppositePath;

    public EdgeScissors(){
        super();
    }

    public void setMiddlePath(final Point[] path){
        this.middlePath = path;
    }

    /*public Roi getEdgeRoi(){

        //First Approach follows points generated from the normal at each point in the line.
        Point[] path = getEdge();
        int[] xArray = new int[path.length];
        int[] yArray = new int[path.length];
        for (int i = 0; i < path.length; i++) {
            xArray[i] = path[i].x;
            yArray[i] = path[i].y;
        }
        PolygonRoi pathRoi = new PolygonRoi(xArray, yArray, path.length, Roi.POLYLINE);
        return pathRoi;
    }*/

    public Roi getEdgeRoi(){

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

        //Set up edge path array
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

    private Point movePointToEdge(Point start, Point direction){
        //Use greedy active contour movement.
        double pointCost = 255 - imageCost.getPixel(start.x, start.y)[0];

        Point newPoint = new Point(start.x + direction.x, start.y + direction.y);

        double newCost = 255 - imageCost.getPixel(newPoint.x, newPoint.y)[0];
        while(newCost < pointCost ){
            pointCost = newCost;
            newPoint = new Point(newPoint.x + direction.x, newPoint.y + direction.y);
            newCost = 255 - imageCost.getPixel(newPoint.x, newPoint.y)[0];
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
            assert(slope != 0);
            System.out.println("Poly slope : " + slope);

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

        this.imageCost = getSobel(saturation, 5);

        setupInfoMatrix();
    }

    /**
     * Creates a shortest path line between series of points of interest.
     * Generates a polygon roi to represent the path.
     * Using shortest path search point to point for each point in array.
     * @param selectedPoints
     * @return polygon roi representing shortest path.
     */
    @Override
    public Roi drawShortestPath(Point[] selectedPoints){
        ArrayList<Point> path = new ArrayList<Point>();

        //Reset the nodes so previous search paths dont interfer.
        reset();

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
                //Reverse to get points in correct order before adding to path.
                Collections.reverse(shortestPath);
                path.addAll(shortestPath);
            }
            //Add the last point.
            path.add(selectedPoints[selectedPoints.length-1]);
        }

        //Convert Arraylist to x and y array for constructing the polygon roi.
        int[] xArray = new int[path.size()];
        int[] yArray = new int[path.size()];
        for (int i = 0; i < path.size(); i++) {
            xArray[i] = path.get(i).x;
            yArray[i] = path.get(i).y;
        }

        PolygonRoi pathRoi = new PolygonRoi(xArray, yArray, path.size(), Roi.POLYLINE);

        return pathRoi;

    }

    /**
     * Traces the shortest path between two points after successful shortest path search.
     * Follows the node previous pointers from end to the start.
     * Only works if shortest path search has been done using the two points of interest.
     * @param start - start node point.
     * @param goal - end node point.
     * @return list of points tracing shortest path from start to goal, null if path unknown.
     */
    @Override
    public ArrayList<Point> getShortestPath(Point start, Point goal) {
        Point temp = new Point(goal.x, goal.y);
        ArrayList<Point> path = new ArrayList<Point>();
        while (!temp.equals(start)) {
            Point previous = infoMatrix[temp.y][temp.x].previous;
            if (previous != null) {
                temp.x += previous.x;
                temp.y += previous.y;
                path.add(new Point(temp.x, temp.y));
            }else{
                System.err.println("Shortest path between two points has not been explored");
                return null;
            }
        }
        System.out.println("getShortestPath returns : \n" + path);
        return path;
    }

    /**
     * A*  search for shortest path between start and goal.
     * Greedy part cost is the intensity of the imageCost channel in the image.
     * Heuristic is the sum of x and y distance to the goal.
     * @param start
     * @param goal
     */
    @Override
    public void shortestPathSearch(Point start, Point goal){

        PriorityQueue<PixelNode> pq = new PriorityQueue<PixelNode>();

        //Setup first node
        PixelNode seed = infoMatrix[start.y][start.x];
        seed.cost = 0;

        pq.add(seed);

        while(!pq.isEmpty()){
            PixelNode node = pq.poll();

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
                if(neighbour.x > 0 && neighbour.y > 0 && neighbour.x < imageCost.getWidth() && neighbour.y < imageCost.getHeight()) {

                    PixelNode nextNode = infoMatrix[neighbour.y][neighbour.x];

                    if (nextNode.state == State.INITIAL) {
                        //Check cost and update if less
                        double totalcost = node.cost + calculateLinkCost(node.getPoint(), nextNode.getPoint(), goal);
                        nextNode.cost = totalcost;
                        nextNode.previous = neighbours[8 - i];
                        nextNode.state = State.ACTIVE;
                        pq.add(nextNode);

                    } else if (nextNode.state == State.ACTIVE) {
                        //Do nothing
                        double totalcost = node.cost + calculateLinkCost(node.getPoint(), nextNode.getPoint(),goal);
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

    /* Calculates the path cost between two nodes for the shortest path search.
     * Requires knowledge of goal for distance heuristic.
     *
     * @param one - current point
     * @param two - next point on path
     * @param goal - end point in path search
     * @return the cost of moving from one to two.
     */
    private double calculateLinkCost(Point one, Point two, Point goal) {

        //Apply extra cost to moving diagonal.
        int diagonalTest = Math.abs(two.x-one.x)+ Math.abs(two.y - one.y);
        double diagonalMultiplier = 1.0;
        if (diagonalTest == 2){
            diagonalMultiplier = 1.41;
        }

        int distanceFromEnd = Math.abs(goal.y-two.y)+Math.abs(goal.x-goal.x);
        int grayIntensity = imageCost.getPixel(two.x, two.y)[0];

        return ((255-grayIntensity) + distanceFromEnd)*diagonalMultiplier;

    }

}
