package fasciaSegmentation;

import ij.IJ;
import ij.ImagePlus;
import ij.gui.PolygonRoi;
import ij.gui.Roi;
import ij.gui.StackWindow;
import ij.plugin.filter.Convolver;
import ij.plugin.filter.GaussianBlur;
import ij.process.FloatProcessor;
import ij.process.ImageConverter;
import ij.process.ImageProcessor;

import java.awt.*;
import java.util.*;

/**
 * Intelligent Scissors for selecting Fascia strands.
 * Uses intelligent scissors approach of doing shortest path search through image cost graph.
 * Shortest path done using A* search with euclidean distance heuristic.
 * Image cost in the graph is the intensity of the imageCost channel.
 * @author Alexis Barltrop
 */
public class IntelligentScissors {

    /** Saturation channel extracted from image*/
    ImagePlus imageCost;
    /** Points used to move between centre and neighbours*/
    Point[] neighbours = new Point[9];
    /** Array of vertices used for the search*/
    PixelNode[][] infoMatrix = null;
    /** Flags if scissor active of not*/
    boolean scissorActive = false;

    /** Constructor */
    public IntelligentScissors(){
        setupNeighbours();
    }

    /**
     * Alternative constructor for directly setting image
     * @param src - current image
     */
    public IntelligentScissors(ImagePlus src){
        this();
        setImage(src);
    }

    /**
     * Resets all the path costs and directions in the node array.
     */
    public void reset(){
        setupInfoMatrix();
    }

    /**
     *  Converts RGB image into imageCost channel image used for scissors.
     *
     * @param src - RGB image of slice
     */
    public void setImage(ImagePlus src){
        ImagePlus temp = (ImagePlus) src.clone();
        ImageConverter ic = new ImageConverter(temp);
        ic.convertToHSB();
        temp.setSlice(2);
        ImagePlus saturation = new ImagePlus();
        saturation.setProcessor(temp.getProcessor());

       /* //Check result in pop up window
        StackWindow satWindow = new StackWindow(saturation);

        //Testing DoG generation
        ic.convertToGray8();
        ImagePlus dog = getDoG(temp, 1,100);
        StackWindow dogWindow = new StackWindow(dog);

        //Sobel on the DoG

        ImagePlus sobel = getSobel(saturation, 5);
        StackWindow sobelWindow = new StackWindow(sobel);*/

        this.imageCost = saturation;
        setupInfoMatrix();
    }

    /**
     * Initialises the node array for shortest path search.
     */
    protected void setupInfoMatrix(){
        infoMatrix = new PixelNode[imageCost.getHeight()][imageCost.getWidth()];
        //System.out.println(infoMatrix.length);
        //System.out.println(infoMatrix[0].length);
        for(int y = 0 ; y < infoMatrix.length; y++){
            for(int x = 0 ; x < infoMatrix[y].length; x++){
                infoMatrix[y][x] = new PixelNode(x, y);
            }
        }

    }

    /**
     * Populates the neighbours array.
     * Adds points for all eight directions and staying in centre.
     */
    private void setupNeighbours(){
        int count = 0;
        for( int i = -1; i <= 1; i++){
            for(int j = -1; j <=1 ; j++){
                neighbours[count] = new Point(i,j);
                count++;
            }
        }
    }

    public ImagePlus getSobel( final ImagePlus originalImage, final float sigma) {
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



    /**
     * Computes difference of gaussian for the given image.
     * Output is signed 32 bit image of the DoG.
     * @param originalImage
     * @param sigma1
     * @param sigma2
     * @return a 32 bit float image of the DoG.
     */
    public ImagePlus getDoG(final ImagePlus originalImage, final float sigma1, final float sigma2) {

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
                IJ.log("Point 1");
                shortestPathSearch(start,end);
                IJ.log("Point 2");
                ArrayList<Point> shortestPath = getShortestPath(start, end);
                IJ.log("Point 3");
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
    public void shortestPathSearch(Point start, Point goal){

        PriorityQueue<PixelNode> pq = new PriorityQueue<PixelNode>();

        //Setup first node
        PixelNode seed = infoMatrix[start.y][start.x];
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

    /**
     * Calculates the path cost between two nodes for the shortest path search.
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
        }

        public PixelNode(int x, int y){
            this.x = x;
            this.y = y;
            state = State.INITIAL;
            previous = null;
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
