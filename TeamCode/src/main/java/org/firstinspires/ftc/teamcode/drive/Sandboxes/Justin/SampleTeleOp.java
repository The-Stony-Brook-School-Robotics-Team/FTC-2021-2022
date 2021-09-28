package org.firstinspires.ftc.teamcode.drive.Sandboxes.Justin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//justin
@TeleOp(name="Justin TeleOp", group="drive")
public class SampleTeleOp extends OpMode {
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    boolean pressingX = false;
    boolean pressingB = false;
    boolean pressingA = false;
    boolean pressingY = false;
    boolean pressingRightBumper = false;
    boolean pressingLeftBumper = false;
    boolean pressingRightDpad = false;
    boolean pressingLeftDpad = false;
    boolean pressingUp = false;
    boolean pressingDown = false;


    private SampleMecanumDrive drive;


    @Override
    public void init() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");


        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Pose2d startPose = new Pose2d(-61, -27, 0);
        //Pose2d startPose = new Pose2d(12, -6, 0);
        //drive.setPoseEstimate(startPose);
    }

    @Override
    public void loop() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -0.3*gamepad1.left_stick_y,
                        -0.3*gamepad1.left_stick_x,
                        -0.3*gamepad1.right_stick_x
                )
        );
        if (!pressingA && gamepad1.a) {
            goForward();
            pressingA = true;
        }
        else if (pressingA && !gamepad1.a) {
            pressingA = false;
        }
    }

    public void goForward() {
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                .forward(25)
                .build());
        drive.update();
    }
}
