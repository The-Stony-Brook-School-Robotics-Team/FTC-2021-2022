package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;

@Config
public class OdometryLocalizer extends com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = .738188976; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 9.28; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -0.62; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

   // RevBulkData bulkData;

    //ExpansionHubEx expansionHub;

    public OdometryLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        //expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 4");

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftodom"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightodom"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "duck"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
       // rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()), // L
                encoderTicksToInches(rightEncoder.getCurrentPosition()), // R
                encoderTicksToInches(frontEncoder.getCurrentPosition()) // C
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }
}