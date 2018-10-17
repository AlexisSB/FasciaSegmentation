package fasciaSegmentation;

import ij.IJ;
import ij.ImagePlus;
import ij.gui.ImageWindow;
import ij.gui.PolygonRoi;
import ij.gui.Roi;
import ij.plugin.filter.GaussianBlur;
import ij.process.FloatProcessor;
import ij.process.ImageConverter;
import ij.process.ImageProcessor;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;

public class BrushFasciaFilter {

    public ImagePlus displayImage;
    private ImagePlus differenceOfGaussisan;
    private ImagePlus saturation;

    /**
     * Constructor for brush tool version of fascia finder.
     * Processes image to extract saturation and differenc of gaussian.
     * @param displayImage
     */
    public BrushFasciaFilter(ImagePlus displayImage){
        //Generate DOG and all that
        //And Saturation
        this.displayImage = displayImage.duplicate();
        this.differenceOfGaussisan = getDoG(displayImage, 1,100);
        this.saturation = getSaturation(displayImage);

        ImageWindow saturation = new ImageWindow(this.saturation);
        ImageWindow dog = new ImageWindow(this.differenceOfGaussisan);

    }

    /**
     * Takes in a list of slected roi points and returns the ones that are most likeyl due to the Fascia.
     * @param inputRoi
     * @return
     */
    public Roi filterPoints(Roi inputRoi){

        //TODO Create filter mechanic here.
        Point[] inputPoints = inputRoi.getContainedPoints();
//        System.out.println(Arrays.toString(inputPoints));
        ArrayList<Point> outputPoints = new ArrayList<Point>();

        for (int i = 0 ; i < inputPoints.length; i++ ){
            Point p = inputPoints[i];
            System.out.println("Checking point " + p.x + " " + p.y);
            //Red
            //Blue
            //Green
            //Saturation
            int[] pointSaturation = this.saturation.getPixel(p.x, p.y);
            System.out.println(Arrays.toString(pointSaturation));
            //Hue
            //Brightness
            //differenceOfGaussian
            float pointDoG = this.differenceOfGaussisan.getProcessor().getf(p.x, p.y);
            System.out.println(pointDoG);
        }
        /*
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
         */
        return null;
    }

    //TODO Move DOG getter to a parent class for the Path Finder and Brush Finder
    //TODO Defaults for tht esigma parameters??
     /** Computes difference of gaussian for the given image.
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

    public ImagePlus getSaturation(final ImagePlus originalImage){
        ImagePlus temp = originalImage.duplicate();
        ImageConverter ic = new ImageConverter(temp);
        ic.convertToHSB();
        temp.setSlice(2);
        ImagePlus output = new ImagePlus("saturation", temp.getProcessor());
        return output;
    }


    public void reset(){
        displayImage = null;
        saturation = null;
        differenceOfGaussisan = null;
    }

    public void setDisplayImage(ImagePlus displayImage) {
        this.displayImage = displayImage;


    }




}
