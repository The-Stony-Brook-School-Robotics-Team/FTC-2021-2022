package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.BearsUtil.T265Controller;
import org.firstinspires.ftc.teamcode.common.BearsUtil.T265Exception;

/**
 * This class is a RR Localizer that uses the Intel T265 Camera.
 * @author Marc N
 * @version 12.0.1
 */
public class T265Localizer implements Localizer {

    /**
     * This is the camera controller object, which is used to manipulate the camera.
     */
    T265Controller camCtrl;

    /**
     * This is the main constructor, which simply initializes the T265Controller.
     * @param hardwareMap the HardwareMap Object provided through SampleMecanumDrive.
     */
    public T265Localizer(HardwareMap hardwareMap){
        camCtrl = new T265Controller(hardwareMap,null);
    }


    /**
     * This method returns position using the camera.
     * @return the current robot position in RR <code>Pose2d</code> format.
     */
    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return camCtrl.getIntelPos();
    }


    /**
     * This method sets the pose given in the parameter.
     * @param pose2d the new Pose of the Robot.
     */
    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        T265Controller.intelCam.setPose(T265Controller.reverseConvert265PosToRR(pose2d));
    }


    /**
     * This method returns velocity using the camera.
     * @return the current robot velocity in RR <code>Pose2d</code> format.
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
         ChassisSpeeds sped = T265Controller.intelCam.getLastReceivedCameraUpdate().velocity;
         return new Pose2d(sped.vxMetersPerSecond, sped.vyMetersPerSecond, sped.omegaRadiansPerSecond);
    }

    @Override
    public void update() {

    }

    /**
     * This method shuts down the camera.
     */
    public void shutDown() {
        T265Controller.shutDown();
    }
}
