package org.firstinspires.ftc.teamcode.Sandboxes.Michael;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.DoubleSupplier;

/**
 * This sample shows how to use dead wheels with external encoders
 * paired with motors that don't require encoders.
 * In this sample, we will use the drive motors' encoder
 * ports as they are not needed due to not using the drive encoders.
 * The external encoders we are using are REV through-bore.
 */
@TeleOp
//@Disabled
public class deadwheels extends LinearOpMode {

    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 12.75;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = -8.7;
    public static final double odomRadius = 1;
    public static final double WHEEL_DIAMETER = 2.0;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MotorEx lf, rf, lb, rb;
    private MecanumDrive driveTrain;
    private Motor intakeLeft, intakeRight, liftLeft, liftRight;
    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;

    private Waypoint p1;
    private Waypoint p2;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = new MotorEx(hardwareMap, "lf");
        rf = new MotorEx(hardwareMap, "leftodom");
        lb = new MotorEx(hardwareMap, "backodom");
        rb = new MotorEx(hardwareMap, "rightodom");

        driveTrain = new MecanumDrive(lf, rf, lb, rb);

        //intakeLeft = new Motor(hardwareMap, "intake_left");
        //intakeRight = new Motor(hardwareMap, "intake_right");
        //liftLeft = new Motor(hardwareMap, "lift_left");
        //liftRight = new Motor(hardwareMap, "lift_right");
        // Here we set the distance per pulse of the odometers.
        // This is to keep the units consistent for the odometry.
        leftOdometer = rf.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = rb.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = lb.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        DoubleSupplier leftValue, rightValue, horizontalValue;

        leftValue = () -> (convertOdomTickToRobotInches(leftOdometer.getPosition()));
        rightValue = () -> (convertOdomTickToRobotInches(rightOdometer.getPosition()));
        horizontalValue = () -> (convertOdomTickToRobotInches(centerOdometer.getPosition()));

        //odometry = new HolonomicOdometry(
          //      leftOdometer::getDistance,
            //    rightOdometer::getDistance,
              //  centerOdometer::getDistance,
                //TRACKWIDTH, CENTER_WHEEL_OFFSET
        //);
        odometry = new HolonomicOdometry(
                leftValue,
                rightValue,
                horizontalValue,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        odometry.update(0, 0, 0);

        //p1 = new GeneralWaypoint(10, 10);
        //p2 = new GeneralWaypoint(10, 10);


        Waypoint start = new StartWaypoint(0, 0);
        Waypoint end = new EndWaypoint(20, 20, 0, 0.7,
                0.5, 30, 0.8, 1);

        Path path1 = new Path(start, end);
        path1.init();

        telemetry.addData("currPoss", odometry.getPose());
        telemetry.update();

        waitForStart();

        path1.followPath(driveTrain, odometry);
        telemetry.addData("currPoss", odometry.getPose());
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {


            odometry.updatePose(); // update the position
            telemetry.addData("test", "test");
            telemetry.addData("POSE!!!!", odometry.getPose());
            telemetry.update();

        }
    }

    public static double convertOdomTickToRobotInches(double ticks) {
        return ticks/(8192*odomRadius*2*Math.PI); // tested experimentally with all 3 odoms on 9/16/2021
    }

}