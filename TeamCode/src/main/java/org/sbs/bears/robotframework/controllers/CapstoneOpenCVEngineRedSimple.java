package org.sbs.bears.robotframework.controllers;

import static org.sbs.bears.robotframework.controllers.OpenCVController.doAnalysisMaster;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.sbs.bears.robotframework.enums.DuckPosition;

import java.util.ArrayList;


/**
 * This class is the OpenCV Pipeline for Analyzing TSE Position from Blue side, near the Duck.
 * @author Marc N
 * @version 5.1
 */
@Config
public class CapstoneOpenCVEngineRedSimple extends DuckOpenCVEngine {
    // MARK - Class Variables

    public CapstoneOpenCVEngineRedSimple() {
        super();
        Log.d("RedSimpleCapstoneOpenCVController","Init req");
    }

    /**
     * This variable represents the width of the Camera Stream.
     */
    static final int STREAM_WIDTH = 1920;
    /**
     * This variable represents the height of the Camera Stream.
     */
    static final int STREAM_HEIGHT = 1080;

    /**
     * This variable is the Semaphore.
     * It makes sure concurrent threads don't modify and read variables at the same time.
     */
    static volatile Object semaphore = new Object();



    /**
     * This variable represents the Duck's Position on the barcode.
     * This variable is made to be read from outside (hence its "protected" state).
     * The "volatile" keyword makes it better suited for use with multithreading.
     */
    protected static volatile DuckPosition position = DuckPosition.NA;


    /**
     * This variable represents a Blue color.
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    /**
     * This variable represents a Green color.
     */
    static final Scalar GREEN = new Scalar(0, 255, 0);
    /**
     * This variable represents a Red color.
     */
    static final Scalar RED = new Scalar(255, 0, 0);

    /**
     * This variable represents the WIDTH of the rectangle about barcode A.
     */
    public static int WidthRectA = 130;
    /**
     * This variable represents the HEIGHT of the rectangle about barcode A.
     */
    public static int HeightRectA = 110;
    /**
     * This variable represents the XOffset of the rectangle about barcode A.
     */
    public static int XOffsetA = -40; // -500
    /**
     * This variable represents the YOffset of the rectangle about barcode A.
     */
    public static int YOffsetA = 425; // 250
    /**
     * This variable represents the Top Left Anchor of the rectangle about barcode A.
     */
    static final Point RectATopLeftAnchor = new Point((STREAM_WIDTH - WidthRectA) / 2 + XOffsetA, ((STREAM_HEIGHT - HeightRectA) / 2) + YOffsetA);

    /**
     * This variable represents the WIDTH of the rectangle about barcode B.
     */
    public static int WidthRectB = 130;
    /**
     * This variable represents the HEIGHT of the rectangle about barcode B.
     */
    public static int HeightRectB = 110;
    /**
     * This variable represents the XOffset of the rectangle about barcode B.
     */
    public static int XOffsetB = 500;
    /**
     * This variable represents the YOffset of the rectangle about barcode B.
     */
    public static int YOffsetB = 425;
    /**
     * This variable represents the Top Left Anchor of the rectangle about barcode B.
     */
    static final Point RectBTopLeftAnchor = new Point((STREAM_WIDTH - WidthRectB) / 2 + XOffsetB, ((STREAM_HEIGHT - HeightRectA) / 2) + YOffsetB);

    /**
     * This variable represents the WIDTH of the rectangle about barcode C.
     */
    public static  int WidthRectC = 130;
    /**
     * This variable represents the HEIGHT of the rectangle about barcode C.
     */
    public static  int HeightRectC = 110;
    /**
     * This variable represents the XOffset of the rectangle about barcode C.
     */
    public static int XOffsetC = 850;
    /**
     * This variable represents the YOffset of the rectangle about barcode C.
     */
    public static int YOffsetC = 425;
    /**
     * This variable represents the Top Left Anchor of the rectangle about barcode C.
     */
    static final Point RectCTopLeftAnchor = new Point((STREAM_WIDTH - WidthRectB) / 2 + XOffsetC, ((STREAM_HEIGHT - HeightRectA) / 2) + YOffsetC);



    /**
     * This variable represents the Top Left Corner of the rectangle about barcode A.
     */
    Point RectATLCorner = new Point(
            RectATopLeftAnchor.x,
            RectATopLeftAnchor.y);
    /**
     * This variable represents the Bottom Right Corner of the rectangle about barcode A.
     */
    Point RectABRCorner = new Point(
            RectATopLeftAnchor.x + WidthRectA,
            RectATopLeftAnchor.y + HeightRectA);

    /**
     * This variable represents the Top Left Corner of the rectangle about barcode B.
     */
    Point RectBTLCorner = new Point(
            RectBTopLeftAnchor.x,
            RectBTopLeftAnchor.y);
    /**
     * This variable represents the Bottom Right Corner of the rectangle about barcode B.
     */
    Point RectBBRCorner = new Point(
            RectBTopLeftAnchor.x + WidthRectB,
            RectBTopLeftAnchor.y + HeightRectB);

    /**
     * This variable represents the Top Left Corner of the rectangle about barcode C.
     */
    Point RectCTLCorner = new Point(
            RectCTopLeftAnchor.x,
            RectCTopLeftAnchor.y);
    /**
     * This variable represents the Bottom Right Corner of the rectangle about barcode C.
     */
    Point RectCBRCorner = new Point(
            RectCTopLeftAnchor.x + WidthRectC,
            RectCTopLeftAnchor.y + HeightRectC);


    /**
     * This variable contains the image that the camera takes of its view
     * in the YCrCb color space.
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    Mat YCrCb = new Mat();
    /**
     * This variable contains only the Y component of the image that the camera takes of its view
     * in the YCrCb color space.
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    Mat Y = new Mat();
    /**
     * This variable contains the sub-image that the camera takes of the rectangle about barcode A (Y component only).
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    Mat RectA_Y;
    /**
     * This variable contains the sub-image that the camera takes of the rectangle about barcode B (Y component only).
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    Mat RectB_Y;
    /**
     * This variable contains the sub-image that the camera takes of the rectangle about barcode C (Y component only).
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    Mat RectC_Y;

    /**
     * This variable contains the average value of Y in the rectangle about barcode A.
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    volatile int avgAY;
    /**
     * This variable contains the average value of Y in the rectangle about barcode B.
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    volatile int avgBY;
    /**
     * This variable contains the average value of Y in the rectangle about barcode C.
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    volatile int avgCY;

    /**
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable,
     * and also creates the submats of each rectangle (A,B,C).
     * @param input the Mat that is provided by the OpenCV subsystem.
     */
    public void convertY(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb); // convert to YCrCb
        ArrayList<Mat> yCrCbChannels = new ArrayList<Mat>(3);
        Core.split(YCrCb, yCrCbChannels); // split into Y, Cr, and Cb
        Y = yCrCbChannels.get(1);
        RectA_Y = Y.submat(new Rect(RectATLCorner, RectABRCorner));
        RectB_Y = Y.submat(new Rect(RectBTLCorner, RectBBRCorner));
        RectC_Y = Y.submat(new Rect(RectCTLCorner, RectCBRCorner));
    }


    /**
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable,
     * and also creates the submats of each rectangle (A,B,C).
     * @param firstFrame the Mat that is provided by the OpenCV subsystem.
     */
    @Override
    public void init(Mat firstFrame) {
       convertY(firstFrame);
        Log.d("BlueFullDuckOpenCVController","Init complete");
    }

    /**
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable,
     * and also creates the submats of each rectangle (A,B,C).
     * Then, it takes the average values of the Y on each mat as an analysis.
     * It subsequently releases the mats in memory to prevent leakage.
     * It decides which barcode the duck is on by finding the two with the
     * smallest difference, then choosing the third.
     * @param input the Mat that is provided by the OpenCV subsystem.
     * @return the input mat with rectangles representing the three barcode areas drawn on it.
     */
    @Override
    public Mat processFrame(Mat input) {
        Log.d("BlueFullDuckOpenCVController","Start ProcessFrame");
        if(doAnalysisMaster) {
           convertY(input);
           synchronized (semaphore) {
               avgAY = (int) Core.mean(RectA_Y).val[0];
               avgBY = (int) Core.mean(RectB_Y).val[0];
               avgCY = (int) Core.mean(RectC_Y).val[0];
           }
           // Release the mats! Otherwise there are memory leaks and it crashes.
           RectA_Y.release();
           RectB_Y.release();
           RectC_Y.release();
           Y.release();
           YCrCb.release();
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

       if(doAnalysisMaster) {
           int dAB = Math.abs(avgAY-avgBY);
           int dAC = Math.abs(avgAY-avgCY);
           int dBC = Math.abs(avgBY-avgCY);

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
       }
        Log.d("BlueFullDuckOpenCVController","End ProcessFrame");
        Log.d("BlueFullDuckOpenCVController","A: " + avgAY + " B: " + avgBY + " C: " +  avgCY);

        if(avgAY != 0 && avgBY != 0 && avgCY != 0) {
            doAnalysisMaster = false;
        }
        return input;
    }

    /**
     * This method returns the average 'Y' value on barcode A using a semaphore to make sure
     * concurrent threads don't overwrite each other.
     * @return the average 'Y' value on barcode A
     */
    public int getAYanalysis() {
        synchronized (semaphore) {return avgAY;}
    }
    /**
     * This method returns the average 'Y' value on barcode B using a semaphore to make sure
     * concurrent threads don't overwrite each other.
     * @return the average 'Y' value on barcode B
     */
    public int getBYanalysis() {
         synchronized (semaphore) {return avgBY;}
    }
    /**
     * This method returns the average 'Y' value on barcode C using a semaphore to make sure
     * concurrent threads don't overwrite each other.
     * @return the average 'Y' value on barcode C
     */
    public int getCYanalysis() {
         synchronized (semaphore) {return avgCY;}
    }
    /**
     * This method returns the position of the duck on the barcode
     * using a semaphore to make sure concurrent threads don't
     * overwrite each other.
     * @return the duck position on the barcode
     */
    public DuckPosition getPosition() {
        synchronized (semaphore) {return position;}
    }

}