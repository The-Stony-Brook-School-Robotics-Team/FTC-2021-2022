package org.firstinspires.ftc.teamcode.BearsUtil;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class T265Controller {
    static T265Camera intelCam;
    Transform2d camToRobot;
    private double whichX;
    private double dX;
    private double whichY;
    private double dY;
    private double whichH;
    private double dH;
    public T265Controller(HardwareMap hwMap, Telemetry telemetry) {
        camToRobot = new Transform2d(new Translation2d(0.025,-0.08),new Rotation2d());
        intelCam = new T265Camera(camToRobot,0.1,hwMap.appContext);
        try {intelCam.start();
            sleep(1000);}
        catch (Exception e)
        {
            e.printStackTrace();
            intelCam.stop();
            telemetry.addData("You forgot to stop","the camera!!");
            telemetry.update();
            intelCam.start();
            try {
                sleep(1000);
            } catch (InterruptedException interruptedException) {
                interruptedException.printStackTrace();
            }
            telemetry.addData("Initialized","camera");
            telemetry.update();

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
        com.arcrobotics.ftclib.geometry.Pose2d LastPose = intelCam.getLastReceivedCameraUpdate().pose;
        double x = LastPose.getTranslation().getX() / 0.0254;
        double y = LastPose.getTranslation().getY() / 0.0254;
        double h = LastPose.getHeading();
        return new Pose2d(x,y,h);
    }
    public void shutDown() {
        intelCam.stop();
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
