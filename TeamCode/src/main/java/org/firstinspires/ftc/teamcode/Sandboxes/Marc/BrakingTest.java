package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;

public class BrakingTest extends OpMode {
    RoadRunnerController RRctrl;
    SampleMecanumDrive drive;
    MecanumDrive.MecanumLocalizer encoderLocal;
    DcMotorEx[] motors = new DcMotorEx[4];

    int conteur = 0;

    @Override
    public void init() {
        RRctrl = new RoadRunnerController(hardwareMap,telemetry);
        drive = RRctrl.getDrive();
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        encoderLocal = new MecanumDrive.MecanumLocalizer(drive);
        motors[0] = hardwareMap.get(DcMotorEx.class, "lf");
        motors[1] = hardwareMap.get(DcMotorEx.class, "lb");
        motors[2] = hardwareMap.get(DcMotorEx.class, "rb");
        motors[3] = hardwareMap.get(DcMotorEx.class, "rf");

    }

    @Override
    public void loop() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
        drive.update();
        if(isSkidding())
        {
            stopSkiddingX();
            conteur++;
        }
    }


    public boolean isSkidding()
    {
        if(epsilonEqualsPose(encoderLocal.getPoseEstimate(),drive.getPoseEstimate()))
        {
            return false;
        }
        return true;
    }

    public void stopSkiddingX()
    {
        // set omega to corresponding value with v.

        double velX = drive.getPoseVelocity().getX(); // in/s
        double velY = drive.getPoseVelocity().getY(); // in/s
        double velMag = Math.sqrt(velX*velX+velY*velY); // in/s

        double omega = velMag/DriveConstants.WHEEL_RADIUS; // rad/s

        for (DcMotorEx mot : motors) {
            mot.setVelocity(omega, AngleUnit.RADIANS);
        }
        if(conteur > 20)
        {
            encoderLocal.setPoseEstimate(drive.getPoseEstimate());
        }
    }


    public static boolean epsilonEquals(double one, double two)
    {
        return (Math.abs(two-one) < 10e-2);
    }
    public static boolean epsilonEqualsPose(Pose2d one, Pose2d two)
    {
        return (Math.abs(two.getX()-one.getX()) < 10e-2) &&
                (Math.abs(two.getY()-one.getY()) < 10e-2) &&
                (Math.abs(two.getHeading()-one.getHeading()) < 10e-2);
    }


    public static void print(Object smthn)
    {
        System.out.println(smthn);
    }
}
