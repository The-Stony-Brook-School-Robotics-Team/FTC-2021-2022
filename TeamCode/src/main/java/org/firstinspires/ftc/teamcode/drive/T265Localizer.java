package org.firstinspires.ftc.teamcode.drive;

import android.telecom.TelecomManager;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BearsUtil.T265Controller;

public class T265Localizer implements Localizer {
    T265Controller camCtrl;

    public T265Localizer(HardwareMap hardwareMap) {
        camCtrl = new T265Controller(hardwareMap,null);
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return camCtrl.getIntelPos();
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        T265Controller.intelCam.setPose(T265Controller.reverseConvert265PosToRR(pose2d));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
         ChassisSpeeds sped = T265Controller.intelCam.getLastReceivedCameraUpdate().velocity;
         return new Pose2d(sped.vxMetersPerSecond, sped.vyMetersPerSecond, sped.omegaRadiansPerSecond);
    }

    @Override
    public void update() {

    }
}
