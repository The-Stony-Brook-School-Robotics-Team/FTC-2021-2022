package org.firstinspires.ftc.teamcode.common.util;

import static java.lang.Thread.sleep;

//import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class is a wrapper for controlling and obtaining data from
 * the Intel Realsense™ T265 Camera.
 * @author Marc N.
 * @version 12.0.1
 */
public class T265Controller {

    /**
     * This is the <strong>static</strong> T265Camera object, which represents
     * the camera itself.
     * Note that it is accessible from outside in order to obtain velocity.
     */
    public static T265Camera intelCam;
    /**
     * This is the transformation from the camera to the robot to account for turning
     * and being able to follow trajectories.
     */
    Transform2d camToRobot;
    /**
     * This is the Pose2d object representing current position of the robot.
     */
    Pose2d currentPos;

    /**
     * This is the main constructor of this class. It requires hardwareMap and telemetry objects,
     * and it instantiates the camera object if it has not already been instantiated.
     * @param hwMap the HardwareMap of the robot. Must be provided through the OpMode
     *              instantiating the controller.
     * @param telemetry the Telemetry object; used to send messages to the Driver Station.
     */
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
            // TODO
            // camera not initialized properly.
        }

    }

    /**
     * This method converts FTClib <code>Pose2d</code>s to RoadRunner <code>Pose2d</code>s.
     * @param input the FTClib <code>Pose2d</code>
     * @return the RR <code>Pose2d</code>
     */
    public static Pose2d convert265PosToRR(com.arcrobotics.ftclib.geometry.Pose2d input)
    {
        return new Pose2d(input.getTranslation().getX(),input.getTranslation().getY(),input.getHeading());
    }
    /**
     * This method converts RR <code>Pose2d</code>s to FTClib <code>Pose2d</code>s.
     * @param input the RR <code>Pose2d</code>
     * @return the FTClib <code>Pose2d</code>
     */
    public static com.arcrobotics.ftclib.geometry.Pose2d reverseConvert265PosToRR(Pose2d input)
    {
        return new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(input.getX(),input.getY()),new Rotation2d(input.getHeading()));
    }

    /**
     * This method returns the robot's current position using the T265 in a RR format.
     * @return the RR <code>Pose2d</code> representing current robot position.
     */
    public Pose2d getIntelPos()
    {
        double a = 2.5; // offset x
        double b = 8; // offset y
        com.arcrobotics.ftclib.geometry.Pose2d LastPose = intelCam.getLastReceivedCameraUpdate().pose;
        double h = LastPose.getHeading();
        double x = LastPose.getTranslation().getX() / 0.0254  + a*Math.sin(-h) - b*Math.cos(-h); // use trigonometry to account for camera offset.
        double y = -1*(LastPose.getTranslation().getY() / 0.0254  - a*Math.cos(-h) + b*Math.sin(-h));
       currentPos =  new Pose2d(x,y,-h); // RR
       return currentPos;
    }
    /**
     * This method shuts down the T265, freeing the object and stopping the camera.
     */
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
