package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.common.teleop.v1.misc.Beta;
import org.sbs.bears.coyote.enums.DoNotUse;

@Deprecated
@DoNotUse
@Beta
public class CustomTankLocalizer implements Localizer {

    SampleTankDrive drive;
    boolean useExtHeading = false;
    
    public CustomTankLocalizer(SampleTankDrive drive, boolean useExtHeading)
    {
        this.drive = drive;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return null;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {

    }
}
