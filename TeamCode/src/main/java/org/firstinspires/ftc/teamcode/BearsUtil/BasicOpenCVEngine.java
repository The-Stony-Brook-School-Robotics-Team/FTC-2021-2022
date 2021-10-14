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

    final int PresentThreshold = 127;

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

    /*
     * Working variables
     */
    Mat RectA_Cb;
    Mat RectA_Cr;
    Mat RectA_Y;


    //Mat regionGoal_Cr;
    Mat YCrCb = new Mat();
    Mat Y = new Mat();
    Mat Cr = new Mat();
    Mat Cb = new Mat();
    ///Mat Cr = new Mat();
    int avgA;


    int avgACr;

    int avgAY;


    /*
     * Working variables
     */

    Mat RectB_Cb;
    Mat RectC_Cb;

    Mat RectB_Cr;
    Mat RectC_Cr;

    Mat RectB_Y;
    Mat RectC_Y;

    //Mat regionGoal_Cr;

    ///Mat Cr = new Mat();

    int avgB;
    int avgC;


    int avgBCr;
    int avgCCr;

    int avgBY;
    int avgCY;

    //int avgGoalCr;


    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2HSV);
        ArrayList<Mat> yCrCbChannels = new ArrayList<Mat>(3);
        Core.split(YCrCb, yCrCbChannels);
        //Core.extractChannel(YCrCb, Cb, 1);
        Y = yCrCbChannels.get(0);
        Cr = yCrCbChannels.get(1);
        Cb = yCrCbChannels.get(2);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);

        RectA_Cb = Cb.submat(new Rect(RectATLCorner, RectABRCorner));
        RectB_Cb = Cb.submat(new Rect(RectBTLCorner, RectBBRCorner));
        RectC_Cb = Cb.submat(new Rect(RectCTLCorner, RectCBRCorner));

        RectA_Cr = Cr.submat(new Rect(RectATLCorner, RectABRCorner));
        RectB_Cr = Cr.submat(new Rect(RectBTLCorner, RectBBRCorner));
        RectC_Cr = Cr.submat(new Rect(RectCTLCorner, RectCBRCorner));

        RectA_Y = Y.submat(new Rect(RectATLCorner, RectABRCorner));
        RectB_Y = Y.submat(new Rect(RectBTLCorner, RectBBRCorner));
        RectC_Y = Y.submat(new Rect(RectCTLCorner, RectCBRCorner));

        /////regionGoal_Cr = Cr.submat(new Rect(region1_pointA_goal, region1_pointB_goal));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
          avgA = (int) Core.mean(RectA_Cb).val[0];
        avgB = (int) Core.mean(RectB_Cb).val[0];
        avgC = (int) Core.mean(RectC_Cb).val[0];

        avgACr = (int) Core.mean(RectA_Cr).val[0];
        avgBCr = (int) Core.mean(RectB_Cr).val[0];
        avgCCr = (int) Core.mean(RectC_Cr).val[0];

        avgAY = (int) Core.mean(RectA_Y).val[0];
        avgBY = (int) Core.mean(RectB_Y).val[0];
        avgCY = (int) Core.mean(RectC_Y).val[0];
        //avgGoalCr = (int) Core.mean(regionGoal_Cr).val[0]; // need to fix val[0]




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


        return input;
    }


    public int getAanalysis() {
        return avgA;
    }



    public int getACranalysis() {
        return avgACr;
    }



    public int getAYanalysis() {
        return avgAY;
    }



}