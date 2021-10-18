package org.firstinspires.ftc.teamcode.BearsUtil;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class BasicOpenCVEngine extends OpenCvPipeline {
    static final int STREAM_WIDTH = 1920;
    static final int STREAM_HEIGHT = 1080;
    static volatile Object semaphore = new Object();
    public static boolean doAnalysis = false;
    static volatile DuckPosition position = DuckPosition.NA;

    enum DuckPosition {
        NA,
        A,
        B,
        C
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final int WidthRectA = 130; // for ini rings window
    static final int HeightRectA = 110; // for ini rings window
    static final Point RectATopLeftAnchor = new Point((STREAM_WIDTH - WidthRectA) / 2 + 300, ((STREAM_HEIGHT - HeightRectA) / 2) - 100);

    static final int WidthRectB = 130; // for goal alignment window
    static final int HeightRectB = 110; // for goal alignment window
    static final Point RectBTopLeftAnchor = new Point((STREAM_WIDTH - WidthRectB) / 2 + 200, ((STREAM_HEIGHT - HeightRectA) / 2) - 100);

    static final int WidthRectC = 130; // for goal alignment window
    static final int HeightRectC = 110; // for goal alignment window
    static final Point RectCTopLeftAnchor = new Point((STREAM_WIDTH - WidthRectB) / 2 + 100, ((STREAM_HEIGHT - HeightRectA) / 2) - 100);


    Point RectATLCorner = new Point(
            RectATopLeftAnchor.x,
            RectATopLeftAnchor.y);
    Point RectABRCorner = new Point(
            RectATopLeftAnchor.x + WidthRectA,
            RectATopLeftAnchor.y + HeightRectA);

    Point RectBTLCorner = new Point(
            RectBTopLeftAnchor.x,
            RectBTopLeftAnchor.y);
    Point RectBBRCorner = new Point(
            RectBTopLeftAnchor.x + WidthRectB,
            RectBTopLeftAnchor.y + HeightRectB);

    Point RectCTLCorner = new Point(
            RectCTopLeftAnchor.x,
            RectCTopLeftAnchor.y);
    Point RectCBRCorner = new Point(
            RectCTopLeftAnchor.x + WidthRectC,
            RectCTopLeftAnchor.y + HeightRectC);

    Mat RectA_Y;
    Mat YCrCb = new Mat();
    Mat Y = new Mat();
    int avgA;
    int avgACr;
    volatile int avgAY;
    Mat RectB_Y;
    Mat RectC_Y;
    volatile int avgBY;
    volatile int avgCY;
    public boolean analyzedOnce = false;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void convertY(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2HSV);
        ArrayList<Mat> yCrCbChannels = new ArrayList<Mat>(3);
        Core.split(YCrCb, yCrCbChannels);
        Y = yCrCbChannels.get(0);
    }

    @Override
    public void init(Mat firstFrame) {
       convertY(firstFrame);

        RectA_Y = Y.submat(new Rect(RectATLCorner, RectABRCorner));
        RectB_Y = Y.submat(new Rect(RectBTLCorner, RectBBRCorner));
        RectC_Y = Y.submat(new Rect(RectCTLCorner, RectCBRCorner));
        System.out.println("Submatted");
    }
    @Override
    public Mat processFrame(Mat input) {
       if(doAnalysis) {
           convertY(input);
           synchronized (semaphore) {
               avgAY = (int) Core.mean(RectA_Y).val[0];
               avgBY = (int) Core.mean(RectB_Y).val[0];
               avgCY = (int) Core.mean(RectC_Y).val[0];
           }
       }
        Imgproc.rectangle( // rings
                input, // Buffer to draw on
                RectATLCorner, // First point which defines the rectangle
                RectABRCorner, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Imgproc.rectangle( // rings
                input, // Buffer to draw on
                RectBTLCorner, // First point which defines the rectangle
                RectBBRCorner, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
        Imgproc.rectangle( // rings
                input, // Buffer to draw on
                RectCTLCorner, // First point which defines the rectangle
                RectCBRCorner, // Second point which defines the rectangle
                RED, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

       if(doAnalysis) {

           int dAB = Math.abs(avgAY- avgBY);
           int dAC = Math.abs(avgAY- avgCY);
           int dBC = Math.abs(avgBY- avgCY);

           if (dAB <= 2) {
               synchronized (semaphore) {
                   position = DuckPosition.C;
               }
               System.out.println("Found the duck: C");
           }
           else {
               if (dAC <= 2) {
                   synchronized (semaphore) {
                       position = DuckPosition.B;
                   }
                   System.out.println("Found the duck: B");
               }
               else if (dBC <= 2) {
                   synchronized (semaphore) {
                       position = DuckPosition.A;
                   }
                   System.out.println("Found the duck: A");
               }
           }

           /*if (avgAY > 190) {
               synchronized (semaphore) {
                   position = DuckPosition.A;
               }
               System.out.println("Found the duck: A");
           } else if (avgBY > 190) {
               synchronized (semaphore) {
                   position = DuckPosition.B;
               }
               System.out.println("Found the duck: B");
           } else {
               synchronized (semaphore) {
                   position = DuckPosition.C;
               }
               System.out.println("Found the duck: C");
           }*/
       }
        return input;
    }

    public int getAYanalysis() {
        synchronized (semaphore) {return avgAY;}
    }
    public int getBYanalysis() {
         synchronized (semaphore) {return avgBY;}
    }
    public int getCYanalysis() {
         synchronized (semaphore) {return avgCY;}
    }

    public DuckPosition getPosition() {
        synchronized (semaphore) {return position;}
    }

}