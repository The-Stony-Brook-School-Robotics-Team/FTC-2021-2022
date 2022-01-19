package org.sbs.bears.robotframework.controllers;

import static org.sbs.bears.robotframework.controllers.OpenCVController.doAnalysisMaster;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.sbs.bears.robotframework.enums.DuckPosition;

import java.util.ArrayList;
import java.util.Arrays;


@Config
public class TowerOpenCVEngine extends OpenCvPipeline {
    // MARK - Class Variables

    static int[] removeZeroes(int array[]) {
        ArrayList<Integer> tmp = new ArrayList<>();
        for (int item : array) {
            if(item != 0)
            {
                tmp.add(item);
            }
        }
        int[] newarr = new int[tmp.size()];
        for(int i = 0; i < tmp.size(); i++)
        {
            newarr[i] = tmp.get(i);
        }
        return newarr;
    }

    static int mostFrequent(int arr[])
    {
        if(arr.length == 0) {return 0;}
        // Sort the array
        Arrays.sort(arr);
        int n=arr.length;

        // find the max frequency using linear
        // traversal
        int max_count = 1, res = arr[0];
        int curr_count = 1;

        for (int i = 1; i < n; i++)
        {
            if (arr[i] == arr[i - 1])
                curr_count++;
            else
            {
                if (curr_count > max_count)
                {
                    max_count = curr_count;
                    res = arr[i - 1];
                }
                curr_count = 1;
            }
        }

        // If last element is most frequent
        if (curr_count > max_count)
        {
            max_count = curr_count;
            res = arr[n - 1];
        }

        return res;
    }

    static String printArray(int arr[]) {
        String str = "";
        for (int i : arr)
        {
            str += i + " ";
        }
        return str;
    }


    public TowerOpenCVEngine() {
        super();
        Log.d("BlueFullDuckOpenCVController","Init req");
    }

    static final int STREAM_WIDTH = 480;
    static final int STREAM_HEIGHT = 270;

    /**
     * This variable is the Semaphore.
     * It makes sure concurrent threads don't modify and read variables at the same time.
     */
    static volatile Object semaphore = new Object();


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

    static final Scalar PURPLE = new Scalar(255, 0, 255);

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

    public double sizeOfLayer = -1;
    public double distanceFromCenterOfHub = -1;

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
        Y = yCrCbChannels.get(0);
    }

    public int[] bestColumnsR; // best column in each row
    public int[] bestColumnsL; // best column in each row
    public int[] bestRowsTop; // best column in each row
    public int[] bestRowsBottom; // best column in each row


    public double limitR = -1;
    public double limitT = -1;
    public double limitL = -1;
    public double limitB = -1;


    public double findSizeOfBottomLayer(Mat input)
    {
        int thresholdBlack = 30;
        bestColumnsL = new int[input.rows()];
        bestColumnsR = new int[input.rows()];
        bestRowsTop = new int[input.cols()];
        bestRowsBottom = new int[input.cols()];
        // iterate over all rows and find the rightmost ones with black below threshold
        for (int row = 0; row < input.rows(); row++) {
            for(int col = 0; col < input.cols(); col++) {
                if(input.get(row,col)[0] < thresholdBlack)
                {
                    bestColumnsR[row] = col; // replace best one.
                }
            }
        }
        // then iterate over all rows and find the leftmost ones with black below threshold
        for (int row = 0; row < input.rows(); row++) {
            for(int col = input.cols()-1; col >= 0; col--) {
                if(input.get(row,col)[0] < thresholdBlack)
                {
                    bestColumnsL[row] = col; // replace best one.
                }
            }
        }

        for (int col = 0; col < input.cols(); col++) {
            for(int row = 0; row < input.rows(); row++) {
                if(input.get(row,col)[0] < thresholdBlack)
                {
                    bestRowsBottom[col] = row; // replace best one.
                }
            }
        }

        for (int col = 0; col < input.cols(); col++) {
            for(int row = input.rows()-1; row >= 0; row--) {
                if(input.get(row,col)[0] < thresholdBlack)
                {
                    bestRowsTop[col] = row; // replace best one.
                }
            }
        }

        bestColumnsR = removeZeroes(bestColumnsR);
        bestColumnsL = removeZeroes(bestColumnsL);
        bestRowsTop = removeZeroes(bestRowsTop);
        bestRowsBottom = removeZeroes(bestRowsBottom);
         limitR = mostFrequent(bestColumnsR);
         limitL = mostFrequent(bestColumnsL);
         limitT = mostFrequent(bestRowsTop);
         limitB = mostFrequent(bestRowsBottom);



        // then take average of leftmost items and average of rightmost items.
        System.out.println("Found some columns and rows");
        System.out.println("best on R: = " + limitR);
        System.out.println("best on L: = " + limitL);
        System.out.println("best on T: = " + limitT);
        System.out.println("best on B: = " + limitB);
        System.out.println("R: = " + printArray(bestColumnsR));
        System.out.println("L: = " + printArray(bestColumnsL));
        System.out.println("B: = " + printArray(bestRowsBottom));
        System.out.println("T: = " + printArray(bestRowsTop));


        return Math.abs(limitR-limitB);
    }
    public double findDistanceFromCenterOfHub(Mat input)
    {
        return -1;
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
        Log.d("TowerOpenCVEngine","Init complete");
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
        Mat B = new Mat();
        if(doAnalysisMaster) {
           //convertY(input);
            ArrayList<Mat> rgbChannels = new ArrayList<Mat>(3);
            Core.split(input, rgbChannels); // split into Y, Cr, and Cb
            B = rgbChannels.get(2);
           this.sizeOfLayer = findSizeOfBottomLayer(B);
           this.distanceFromCenterOfHub = findDistanceFromCenterOfHub(B);
       }
        /*for(int i = 0; i < Y.rows(); i++)
        {
            for(int col : bestColumnsR)
            {
                Imgproc.rectangle(Y,
                        new Point(col,i),
                        new Point(col+1,i+1),RED);
            }
            for(int col : bestColumnsL)
            {
                Imgproc.rectangle(Y,
                        new Point(col,i),
                        new Point(col+1,i+1),BLUE);
            }
        }
        for(int i = 0; i < Y.cols(); i++)
        {
            for(int row : bestRowsTop)
            {
                Imgproc.rectangle(Y,
                        new Point(i,row),
                        new Point(i+1,row+1),GREEN);
            }
            for(int row : bestRowsBottom)
            {
                Imgproc.rectangle(Y,
                        new Point(i,row),
                        new Point(i+1,row+1),PURPLE);
            }
        }*/

        return B;
    }


}