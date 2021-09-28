package org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;
import java.util.HashMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.Map;

/**
 * it might work or it might not, i guess we have to find out 馬
 *
 */
public class CustomTurn {

    private static double currentPower;
    private static double minimumPower = 0.4;
    private static double maximumPower = 0.9;
    private static double multiplier = 1.2;
    private static double delimiter = 0.2;
    private static int limitDifference = 5;
    private static int powerChange = 0;

    /**
     turnTo allows us to turn to the given angle
     @NotNull BNO055IMU imu = The imu for the robot
     @NotNull int angle = The amount of degrees we want to turn
     @NotNull HashMap<String, DcMotor> motors = A HashMap of the current motors
     */
    public static Orientation turn(@NotNull BNO055IMU imu, @NotNull int angle, @NotNull HashMap<String, DcMotor> motors)
    {

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
        return finalOrientation;
    }

    /**
    turnTo allows us to turn to the given angle
     @NotNull BNO055IMU imu = The imu for the robot
     @NotNull int target = The angle we want to turn to
     @NotNull HashMap<String, DcMotor> motors = A HashMap of the current motors
     */
    public static Orientation turnTo(@NotNull BNO055IMU imu, @NotNull int target, @NotNull HashMap<String, DcMotor> motors)
    {
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
        float difference = initialOrientation.firstAngle - target;
        if(initialOrientation.firstAngle + difference == target)
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
                    }
                    // If we are close to the angle we're trying to be at, slow down the robot
                    else
                    {

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
                    }
                }
            }
            // Cut the power to the motors
            rf.setPower(0);
            lb.setPower(0);
        }
        finalOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return finalOrientation;
    }

}
