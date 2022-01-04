package org.firstinspires.ftc.teamcode.sandboxes.Max;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

public class RoadRunnerTrajectories extends LinearOpMode {
DcMotor m1;
double Gain = 0.5;
    SampleMecanumDrive MecanumDrive;

    @Override
    public void runOpMode() throws InterruptedException {

        StandardTrackingWheelLocalizer Tracking = new StandardTrackingWheelLocalizer(hardwareMap);
        Tracking.setPoseEstimate(new Pose2d(0,56,Math.toRadians(0)));

        m1 = hardwareMap.get(DcMotor.class, "m1");
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        MecanumDrive.trajectoryBuilder(new Pose2d(0,56))
                .strafeRight(20)
                .addDisplacementMarker(()->{

                    m1.setPower(Gain);
                    sleep(800);
                    m1.setPower(0);


                })
                .splineTo(new Vector2d(20,0), Math.toRadians(0))
                .build();


        while(true){





        }
    }
}
