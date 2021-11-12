package org.firstinspires.ftc.teamcode.common.BearsUtil;

import static java.lang.Thread.sleep;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class T265Controller {
   public static T265Camera intelCam;
    Transform2d camToRobot;
    private double whichX;
    private double dX;
    private double whichY;
    private double dY;
    private double whichH;
    private double dH;
    Pose2d currentPos;
    Pose2d softPos;

    public T265Controller(HardwareMap hwMap, Telemetry telemetry) {
        camToRobot = new Transform2d(new Translation2d(0,0),new Rotation2d());
        if(intelCam == null){intelCam = new T265Camera(camToRobot,0.1,hwMap.appContext);}
        try {
            intelCam.start();
            sleep(1000);}
        catch (Exception e)
        {
            e.printStackTrace();
            intelCam.stop();
            if(telemetry != null) {
            telemetry.addData("You forgot to stop","the camera!!");
            telemetry.update();}
            intelCam.start();
            try {
                sleep(1000);
            } catch (InterruptedException interruptedException) {
                interruptedException.printStackTrace();
            }
            if(telemetry != null) {
                telemetry.addData("Initialized","camera");
            telemetry.update();}

        }
        currentPos = getIntelPos();
        Pose2d currentPos2 = getIntelPos();
        if(currentPos2.getX() == currentPos.getX()) {
            Log.d("t265controller","camera not sending good data....");
        }

    }
    public static Pose2d convert265PosToRR(com.arcrobotics.ftclib.geometry.Pose2d input)
    {
        return new Pose2d(input.getTranslation().getX(),input.getTranslation().getY(),input.getHeading());
    }
    public static com.arcrobotics.ftclib.geometry.Pose2d reverseConvert265PosToRR(Pose2d input)
    {
        return new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(input.getX(),input.getY()),new Rotation2d(input.getHeading()));
    }
    public Pose2d getIntelPos()
    {
        double a = 2.5;
        double b = 8;
        com.arcrobotics.ftclib.geometry.Pose2d LastPose = intelCam.getLastReceivedCameraUpdate().pose;
        double h = LastPose.getHeading();
        double x = LastPose.getTranslation().getX() / 0.0254  + a*Math.sin(-h) - b*Math.cos(-h);
        double y = -1*(LastPose.getTranslation().getY() / 0.0254  - a*Math.cos(-h) + b*Math.sin(-h));
       currentPos =  new Pose2d(x,y,-h);
       return currentPos;
    }

    public static void shutDown() {
        intelCam.stop();
        intelCam.free();
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}