package org.sbs.bears.robotframework.controllers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.sbs.bears.robotframework.enums.DuckPosition;

import java.util.ArrayList;


/**
 * This class is the OpenCV Pipeline for Analyzing Duck Position.
 * @author Marc N
 * @version 5.1
 */
@Config
public abstract class DuckOpenCVEngine extends OpenCvPipeline {
    // MARK - Class Variables


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
    public static int XOffsetA = -600; // -500
    /**
     * This variable represents the YOffset of the rectangle about barcode A.
     */
    public static int YOffsetA = 400; // 250
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
    public static int XOffsetB = -150;
    /**
     * This variable represents the YOffset of the rectangle about barcode B.
     */
    public static int YOffsetB = 400;
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
    public static int XOffsetC = 325;
    /**
     * This variable represents the YOffset of the rectangle about barcode C.
     */
    public static int YOffsetC = 400;
    /**
     * This variable represents the Top Left Anchor of the rectangle about barcode C.
     */
    static final Point RectCTopLeftAnchor = new Point((STREAM_WIDTH - WidthRectB) / 2 + XOffsetC, ((STREAM_HEIGHT - HeightRectA) / 2) + YOffsetC);

    /**
     * This variable represents the Top Left Corner of the rectangle about barcode A.
     */
    public Point RectATLCorner = new Point(
            RectATopLeftAnchor.x,
            RectATopLeftAnchor.y);
    /**
     * This variable represents the Bottom Right Corner of the rectangle about barcode A.
     */
    public Point RectABRCorner = new Point(
            RectATopLeftAnchor.x + WidthRectA,
            RectATopLeftAnchor.y + HeightRectA);

    /**
     * This variable represents the Top Left Corner of the rectangle about barcode B.
     */
    public Point RectBTLCorner = new Point(
            RectBTopLeftAnchor.x,
            RectBTopLeftAnchor.y);
    /**
     * This variable represents the Bottom Right Corner of the rectangle about barcode B.
     */
    public Point RectBBRCorner = new Point(
            RectBTopLeftAnchor.x + WidthRectB,
            RectBTopLeftAnchor.y + HeightRectB);

    /**
     * This variable represents the Top Left Corner of the rectangle about barcode C.
     */
    public Point RectCTLCorner = new Point(
            RectCTopLeftAnchor.x,
            RectCTopLeftAnchor.y);
    /**
     * This variable represents the Bottom Right Corner of the rectangle about barcode C.
     */
    public Point RectCBRCorner = new Point(
            RectCTopLeftAnchor.x + WidthRectC,
            RectCTopLeftAnchor.y + HeightRectC);


    /**
     * This variable contains the image that the camera takes of its view
     * in the YCrCb color space.
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    public Mat YCrCb = new Mat();
    /**
     * This variable contains only the Y component of the image that the camera takes of its view
     * in the YCrCb color space.
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    public Mat Y = new Mat();
    /**
     * This variable contains the sub-image that the camera takes of the rectangle about barcode A (Y component only).
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    public Mat RectA_Y;
    /**
     * This variable contains the sub-image that the camera takes of the rectangle about barcode B (Y component only).
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    public Mat RectB_Y;
    /**
     * This variable contains the sub-image that the camera takes of the rectangle about barcode C (Y component only).
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    public Mat RectC_Y;

    /**
     * This variable contains the average value of Y in the rectangle about barcode A.
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    volatile public int avgAY;
    /**
     * This variable contains the average value of Y in the rectangle about barcode B.
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    volatile public int avgBY;
    /**
     * This variable contains the average value of Y in the rectangle about barcode C.
     *
     * NOTE: it must be freed every time after it is used in order to save memory and prevent leakage.
     */
    volatile public int avgCY;

    /**
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable,
     * and also creates the submats of each rectangle (A,B,C).
     * @param input the Mat that is provided by the OpenCV subsystem.
     */
    abstract public void convertY(Mat input);


    /**
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable,
     * and also creates the submats of each rectangle (A,B,C).
     * @param firstFrame the Mat that is provided by the OpenCV subsystem.
     */
    @Override
    abstract public void init(Mat firstFrame);

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
    abstract public Mat processFrame(Mat input);

    /**
     * This method returns the average 'Y' value on barcode A using a semaphore to make sure
     * concurrent threads don't overwrite each other.
     * @return the average 'Y' value on barcode A
     */
    abstract public int getAYanalysis();

    /**
     * This method returns the average 'Y' value on barcode B using a semaphore to make sure
     * concurrent threads don't overwrite each other.
     * @return the average 'Y' value on barcode B
     */
    abstract public int getBYanalysis();
    /**
     * This method returns the average 'Y' value on barcode C using a semaphore to make sure
     * concurrent threads don't overwrite each other.
     * @return the average 'Y' value on barcode C
     */
    abstract public int getCYanalysis();
    /**
     * This method returns the position of the duck on the barcode
     * using a semaphore to make sure concurrent threads don't
     * overwrite each other.
     * @return the duck position on the barcode
     */
    abstract public DuckPosition getPosition();

}