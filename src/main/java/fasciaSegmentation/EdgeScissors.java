package fasciaSegmentation;

import ij.ImagePlus;
import ij.gui.PolygonRoi;
import ij.gui.Roi;
import ij.process.ImageConverter;

import java.awt.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.PriorityQueue;

public class EdgeScissors extends IntelligentScissors {

    public EdgeScissors(){
        super();
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
     * Traces the shortest path between two points after successful shortest path search.
     * Follows the node previous pointers from end to the start.
     * Only works if shortest path search has been done using the two points of interest.
     * @param start - start node point.
     * @param goal - end node point.
     * @return list of points tracing shortest path from start to goal, null if path unknown.
     */
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
