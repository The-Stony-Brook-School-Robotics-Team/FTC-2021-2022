package org.firstinspires.ftc.teamcode.BearsUtil;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class OpenCVEngine  extends OpenCvPipeline {
    static final int STREAM_WIDTH = 640;
    static final int STREAM_HEIGHT = 480;

    /*
     * An enum to define the ring amount
     */
    public enum ItemBarcodePlacement
    {
        A,
        B,
        C,
        NONE
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255,0,0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final int WidthRectA = 130; // for ini rings window
    static final int HeightRectA = 110; // for ini rings window
    static final Point RectATopLeftAnchor = new Point((STREAM_WIDTH - WidthRectA) / 2+250, ((STREAM_HEIGHT - HeightRectA) / 2) - 100);

    static final int WidthRectB = 130; // for goal alignment window
    static final int HeightRectB = 110; // for goal alignment window
    static final Point RectBTopLeftAnchor = new Point((STREAM_WIDTH - WidthRectB) / 2 + 200, ((STREAM_HEIGHT - HeightRectA) / 2) - 100);

    static final int WidthRectC = 130; // for goal alignment window
    static final int HeightRectC = 110; // for goal alignment window
    static final Point RectCTopLeftAnchor = new Point((STREAM_WIDTH - WidthRectB) / 2 +150, ((STREAM_HEIGHT - HeightRectA) / 2) - 100);


    final int PresentThreshold = 127; // TODO fix this

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
    Mat RectB_Cb;
    Mat RectC_Cb;
    Mat RectA_Cr;
    Mat RectB_Cr;
    Mat RectC_Cr;
    Mat RectA_Y;
    Mat RectB_Y;
    Mat RectC_Y;


    Mat RectA_Cymk;
    Mat RectB_Cymk;
    Mat RectC_Cymk;

    Mat RectA_cYmk;
    Mat RectB_cYmk;
    Mat RectC_cYmk;

    Mat RectA_cyMk;
    Mat RectB_cyMk;
    Mat RectC_cyMk;

    Mat RectA_cymK;
    Mat RectB_cymK;
    Mat RectC_cymK;

    //Mat regionGoal_Cr;
    Mat YCrCb = new Mat();
    Mat Ycrcb = new Mat();
    Mat Cr = new Mat();
    Mat Cb = new Mat();

    Mat RGB = new Mat();
    Mat R = new Mat();
    Mat G = new Mat();
    Mat B = new Mat();

    Mat CYMK = new Mat();
    Mat C = new Mat();
    Mat Y = new Mat();
    Mat M = new Mat();
    Mat K = new Mat();

    ///Mat Cr = new Mat();
    int avgA;
    int avgB;
    int avgC;

    int avgACr;
    int avgBCr;
    int avgCCr;
    int avgAY;
    int avgBY;
    int avgCY;

    int[] avgACYMK;
    int[] avgBCYMK;
    int[] avgCCYMK;
    //int avgGoalCr;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile ItemBarcodePlacement position = ItemBarcodePlacement.NONE;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToColorSpaces(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2HSV);
        ArrayList<Mat> yCrCbChannels = new ArrayList<Mat>(3);
        Core.split(YCrCb, yCrCbChannels);
        //Core.extractChannel(YCrCb, Cb, 1);
        Ycrcb = yCrCbChannels.get(0);
        Cr = yCrCbChannels.get(1);
        Cb = yCrCbChannels.get(2);

        input.copyTo(RGB);
        ArrayList<Mat> RGBchannels = new ArrayList<Mat>(3);
        Core.split(RGB, RGBchannels);
        //Core.extractChannel(YCrCb, Cb, 1);
        R = RGBchannels.get(0);
        G = RGBchannels.get(1);
        B = RGBchannels.get(2);

        CYMK = convertRGB2CYMK(RGB);
        ArrayList<Mat> CYMKchannels = new ArrayList<Mat>(4);
        Core.split(CYMK, CYMKchannels);
        C = CYMKchannels.get(0);
        Y = CYMKchannels.get(1);
        M = CYMKchannels.get(2);
        K = CYMKchannels.get(3);
    }

    public static Mat convertRGB2CYMK(Mat RGB) {
        ArrayList<Mat> RGBchannels = new ArrayList<Mat>(3);
        Core.split(RGB, RGBchannels);
        //Core.extractChannel(YCrCb, Cb, 1);
        Mat R = RGBchannels.get(0);
        Mat G = RGBchannels.get(1);
        Mat B = RGBchannels.get(2);

        Mat CYMK = new Mat();
        RGB.copyTo(CYMK);

        for (int i = 0; i < RGB.height(); i++) {
            for (int j = 0; j < RGB.cols(); j++)
            {
                double Rprime = R.get(i,j)[0]/255;
                double Gprime = G.get(i,j)[0]/255;
                double Bprime = B.get(i,j)[0]/255;
                double Kresult = 1-Math.max(Rprime,Math.max(Gprime,Bprime));
                double Cresult = (1 - Rprime - Kresult) / (1-Kresult);
                double Mresult = (1 - Gprime - Kresult) / (1-Kresult);
                double Yresult = (1 - Bprime - Kresult) / (1-Kresult);
                CYMK.put(i,j,new double[]{Cresult,Yresult,Mresult,Kresult});
            }
        }

        return CYMK;
    }

    @Override
    public void init(Mat firstFrame)
    {
        inputToColorSpaces(firstFrame);

        RectA_Cb = Cb.submat(new Rect(RectATLCorner, RectABRCorner));
        RectB_Cb = Cb.submat(new Rect(RectBTLCorner, RectBBRCorner));
        RectC_Cb = Cb.submat(new Rect(RectCTLCorner, RectCBRCorner));

        RectA_Cr = Cr.submat(new Rect(RectATLCorner, RectABRCorner));
        RectB_Cr = Cr.submat(new Rect(RectBTLCorner, RectBBRCorner));
        RectC_Cr = Cr.submat(new Rect(RectCTLCorner, RectCBRCorner));

        RectA_Y = Ycrcb.submat(new Rect(RectATLCorner, RectABRCorner));
        RectB_Y = Ycrcb.submat(new Rect(RectBTLCorner, RectBBRCorner));
        RectC_Y = Ycrcb.submat(new Rect(RectCTLCorner, RectCBRCorner));

        RectA_Cymk = C.submat(new Rect(RectATLCorner, RectABRCorner));
        RectB_Cymk = C.submat(new Rect(RectBTLCorner, RectBBRCorner));
        RectC_Cymk = C.submat(new Rect(RectCTLCorner, RectCBRCorner));

        RectA_cYmk = Y.submat(new Rect(RectATLCorner, RectABRCorner));
        RectB_cYmk = Y.submat(new Rect(RectBTLCorner, RectBBRCorner));
        RectC_cYmk = Y.submat(new Rect(RectCTLCorner, RectCBRCorner));

        RectA_cyMk = M.submat(new Rect(RectATLCorner, RectABRCorner));
        RectB_cyMk = M.submat(new Rect(RectBTLCorner, RectBBRCorner));
        RectC_cyMk = M.submat(new Rect(RectCTLCorner, RectCBRCorner));

        RectA_cymK = K.submat(new Rect(RectATLCorner, RectABRCorner));
        RectB_cymK = K.submat(new Rect(RectBTLCorner, RectBBRCorner));
        RectC_cymK = K.submat(new Rect(RectCTLCorner, RectCBRCorner));


        /////regionGoal_Cr = Cr.submat(new Rect(region1_pointA_goal, region1_pointB_goal));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        inputToColorSpaces(input);

        avgA = (int) Core.mean(RectA_Cb).val[0];
        avgB = (int) Core.mean(RectB_Cb).val[0];
        avgC = (int) Core.mean(RectC_Cb).val[0];

        avgACr = (int) Core.mean(RectA_Cr).val[0];
        avgBCr = (int) Core.mean(RectB_Cr).val[0];
        avgCCr= (int) Core.mean(RectC_Cr).val[0];

        avgAY = (int) Core.mean(RectA_Y).val[0];
        avgBY = (int) Core.mean(RectB_Y).val[0];
        avgCY = (int) Core.mean(RectC_Y).val[0];
        //avgGoalCr = (int) Core.mean(regionGoal_Cr).val[0]; // need to fix val[0]
        avgACYMK = new int[]{(int) Core.mean(RectA_Cymk).val[0],(int) Core.mean(RectA_cYmk).val[0],(int) Core.mean(RectA_cyMk).val[0],(int) Core.mean(RectA_cymK).val[0]};
        avgBCYMK = new int[]{(int) Core.mean(RectB_Cymk).val[0],(int) Core.mean(RectB_cYmk).val[0],(int) Core.mean(RectB_cyMk).val[0],(int) Core.mean(RectB_cymK).val[0]};
        avgCCYMK = new int[]{(int) Core.mean(RectC_Cymk).val[0],(int) Core.mean(RectC_cYmk).val[0],(int) Core.mean(RectC_cyMk).val[0],(int) Core.mean(RectC_cymK).val[0]};



        position = ItemBarcodePlacement.NONE; // Record our analysis
        if (avgA >= PresentThreshold) {
            position = ItemBarcodePlacement.A;
        } else  if (avgB >= PresentThreshold) {
            position = ItemBarcodePlacement.B;
        } else  {
             position = ItemBarcodePlacement.C;
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



        return input;
    }

    public ItemBarcodePlacement getWhichBarcode()
    {
        return position;
    }
    public int getAanalysis()
    {
        return avgA;
    }
    public int getBanalysis()
    {
        return avgB;
    }
    public int getCanalysis()
    {
        return avgC;
    }
    public int getACranalysis()
    {
        return avgACr;
    }
    public int getBCranalysis()
    {
        return avgBCr;
    }
    public int getCCranalysis()
    {
        return avgCCr;
    }
    public int getAYanalysis()
    {
        return avgAY;
    }
    public int getBYanalysis()
    {
        return avgBY;
    }
    public int getCYanalysis()
    {
        return avgCY;
    }

    public int[] getACYMKanalysis()
    {
        return avgACYMK;
    }
    public int[] getBCYMKanalysis()
    {
        return avgBCYMK;
    }
    public int[] getCCYMKanalysis()
    {
        return avgCCYMK;
    }

}
