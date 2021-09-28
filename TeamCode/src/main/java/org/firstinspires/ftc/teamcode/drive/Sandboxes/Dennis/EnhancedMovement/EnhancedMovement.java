package org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement;

// Qualcomm
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// First Inspires
import org.apache.commons.math3.exception.NullArgumentException;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// EnhancedMovement
import org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.exceptions.MovementException;
import org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.geometry.Point;
import org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.profiles.MotionProfile;
import org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.annotations.Testing;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;

// TODO: Finish moveTo functions
@Testing
/**
 * The movement class that will be used with FTCLIB
 */
public class EnhancedMovement {

    // positioning
    private static Point startPoint;
    private static Point currentPoint;

    // imu and hwMp
    private static BNO055IMU imu;
    private static HardwareMap hwMp;

    // Motors
    private static DcMotor lf;
    private static DcMotor rf;
    private static DcMotor lb;
    private static DcMotor rb;
    private static HashMap<String, DcMotor> motors = new HashMap<String, DcMotor>();

    // vars
    private static double currentPower;
    private static double minimumPower = 0.4;
    private static double maximumPower = 0.9;
    private static double multiplier = 1.2;
    private static double delimiter = 0.2;
    private static int limitDifference = 5;
    private static int powerChange = 0;

    /**
     * Just a constructor that you can call to initialize everything
     * @param hardwareMap is the hardwaremap
     * @param telemetry is the telemetry module
     */
    public EnhancedMovement(@NotNull HardwareMap hardwareMap, @NotNull Telemetry telemetry)
    {
        // give ours values
        hwMp = hardwareMap;
        imu = hwMp.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imu.initialize(imuParameters);

        // retrieve la motors
        lf = hwMp.get(DcMotor.class, "lf");
        rf = hwMp.get(DcMotor.class, "rf");
        lb = hwMp.get(DcMotor.class, "lb");
        rb = hwMp.get(DcMotor.class, "rb");
        motors.put("lf", lf);
        motors.put("rf", rf);
        motors.put("lb", lb);
        motors.put("rb", rb);

        // define our start point
        startPoint = new Point(0, 0, 0);
        currentPoint = startPoint;
    }

    /**
     * Our basic moveTo without any type of smoothing, literally just **GO**
     * @param point is the point we want to go to
     * @throws MovementException is the exception we throw if we don't make it to the destination
     */
    public static void moveTo(@NotNull Point point) throws MovementException {
        // i know, don't say anything
        if(point == null) { throw new NullArgumentException(); }
        // get our orientations
        Orientation initialOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        Orientation currentOrientation = initialOrientation;
        // turn
        float turnTarget = currentPoint.heading + point.heading;
        currentOrientation = turn(turnTarget);
        // make sure we got the angle right
        if(currentOrientation.firstAngle == point.heading)
        {

        }
        else
        {
            throw new MovementException();
        }




    }

    /**
     * The whole idea is to move using something that can be more related to a PID for smoothing out turns and overall movement
     * @param point = Destination point
     * @param motionProfile = Motion Profile that will be applied to the motors
     * @throws MovementException is the exception thrown if we don't get to a position properly
     */
    // moveTo using the new motionProfile
    @Testing
    public static void moveTo(@NotNull Point point, @NotNull MotionProfile motionProfile) throws MovementException {
        motionProfile.MOTOR_VELO_MULTIPLIER = 1;
        // i know, don't say anything
        if(point == null) { throw new NullArgumentException(); }
        // get our orientations
        Orientation initialOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        Orientation currentOrientation = initialOrientation;
        // turn
        float turnTarget = currentPoint.heading + point.heading;
        currentOrientation = turn(turnTarget);
        // make sure we got the angle right
        if(currentOrientation.firstAngle == point.heading)
        {

        }
        else
        {
            throw new MovementException();
        }




    }

    /**
     * Turn function that will return a final imu orientation using XYZ **NOT ZYX**
     * @param angle is the amount we want to turn
     * @return will return the final robot / imu orientation
     * @throws MovementException will throw if we don't accurately turn
     */
    public static Orientation turn(@NotNull float angle) throws MovementException {

        // Determine Beginning Orientations
        Orientation finalOrientation;
        Orientation initialOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        // Map Out Motors
        DcMotor lf = motors.get("lf");
        DcMotor rb = motors.get("rb");
        DcMotor rf = motors.get("rf");
        DcMotor lb = motors.get("lb");
        // Set the ZeroPower modes
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Determine which side is easier to turn to
        //if(initialOrientation.firstAngle - angle < initialOrientation.firstAngle + angle)
        float target = initialOrientation.firstAngle + angle;
        if(initialOrientation.firstAngle + angle == target)
        {
            // Turn Motors LF and RB <---
            if(lf.getPower() != minimumPower) { lf.setPower(minimumPower); }
            if(rb.getPower() != minimumPower) { rb.setPower(-minimumPower); }
            currentPower = minimumPower;
            // My attempt at pid
            while(initialOrientation.firstAngle != target)
            {
                Orientation currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                // Check if we can up the speed of the turn
                if(currentPower < maximumPower) {
                    // Check if the angle different is large enough to be able to up the speed
                    if (initialOrientation.firstAngle - currentOrientation.firstAngle > limitDifference)
                    {
                        // Get the current power and multiply it with the multiplier
                        lf.setPower(currentPower * multiplier);
                        rb.setPower(currentPower * multiplier);
                        currentPower = currentPower * multiplier;
                        powerChange++;
                    }
                    // If we are close to the angle we're trying to be at, slow down the robot
                    else
                    {
                        lf.setPower(currentPower * delimiter);
                        rb.setPower(currentPower * delimiter);
                        currentPower = currentPower * delimiter;
                        powerChange = 0;
                    }
                }
                else
                {
                    if(initialOrientation.firstAngle - currentOrientation.firstAngle < limitDifference)
                    {
                        lf.setPower(currentPower * delimiter);
                        rb.setPower(currentPower * delimiter);
                        currentPower = currentPower * delimiter;
                        powerChange = 0;
                    }
                }
            }
            // Cut the power to the motors
            lf.setPower(0);
            rb.setPower(0);
        }
        else
        {
            // Turn Motors RF and LB --->
            if(rf.getPower() != minimumPower) { rf.setPower(minimumPower); }
            if(lb.getPower() != minimumPower) { lb.setPower(-minimumPower); }
            currentPower = minimumPower;
            // My attempt at pid
            while(initialOrientation.firstAngle != target) {
                Orientation currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                // Check if we can up the speed of the turn
                if (currentPower < maximumPower) {
                    // Check if the angle different is large enough to be able to up the speed
                    if (initialOrientation.firstAngle - currentOrientation.firstAngle > limitDifference) {
                        // Get the current power and multiply it with the multiplier
                        rf.setPower(currentPower * multiplier);
                        lb.setPower(currentPower * multiplier);
                        currentPower = currentPower * multiplier;
                        powerChange++;
                    }
                    else
                    {
                        rf.setPower(currentPower * delimiter);
                        lb.setPower(currentPower * delimiter);
                        currentPower = currentPower * delimiter;
                        powerChange = 0;
                    }
                }
                else
                {
                    if(initialOrientation.firstAngle - currentOrientation.firstAngle < limitDifference)
                    {
                        lf.setPower(currentPower * delimiter);
                        rb.setPower(currentPower * delimiter);
                        currentPower = currentPower * delimiter;
                        powerChange = 0;
                    }
                }
            }
            // Cut the power to the motors
            rf.setPower(0);
            lb.setPower(0);
        }
        finalOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        if(finalOrientation.firstAngle != target) { throw new MovementException(); }
        return finalOrientation;
    }
}




