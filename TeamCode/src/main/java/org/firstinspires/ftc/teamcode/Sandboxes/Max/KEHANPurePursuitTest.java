package org.firstinspires.ftc.teamcode.Sandboxes.Max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.function.DoubleSupplier;

@TeleOp
public class KEHANPurePursuitTest extends LinearOpMode {

    private MotorEx LeftEncoder;
    private MotorEx RightEncoder;
    private MotorEx CentralEncoder;

    private MotorEx lf;
    private MotorEx rf;
    private MotorEx lb;
    private MotorEx rb;
    private MecanumDrive Drivers;
    //private SampleMecanumDrive ControlledDrives;
    private int ButtonY = 0;
    FtcDashboard Graph;

    private final double DistancePerPulse = Math.PI*2.0/8192;
    @Override
    public void runOpMode() throws InterruptedException {
        //ControlledDrives = new SampleMecanumDrive(hardwareMap);
        //ControlledDrives.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         lf = new MotorEx(hardwareMap, "lf");
         rf = new MotorEx(hardwareMap, "leftodom");
         lb = new MotorEx(hardwareMap, "backodom");
         rb = new MotorEx(hardwareMap, "rightodom");
        // rb.setInverted(true);
        LeftEncoder = new MotorEx(hardwareMap, "leftodom");
        CentralEncoder = new MotorEx(hardwareMap, "backodom");
        RightEncoder = new MotorEx(hardwareMap, "rightodom");
        RightEncoder.setInverted(true);
        Drivers = new MecanumDrive(lf, rf, lb, rb);
        //ControlledDrives = new SampleMecanumDrive(hardwareMap);
        //Drivers;
        LeftEncoder.setDistancePerPulse(DistancePerPulse);
        RightEncoder.setDistancePerPulse(DistancePerPulse);
        CentralEncoder.setDistancePerPulse(DistancePerPulse);
        LeftEncoder.set(0);
        RightEncoder.set(0);
        CentralEncoder.set(0);
        LeftEncoder.resetEncoder();
        RightEncoder.resetEncoder();
        CentralEncoder.resetEncoder();

        waitForStart();



        DoubleSupplier LP,RP,CP;
        double TicksPerInch = 8192*4*Math.PI;
        LeftEncoder.resetEncoder();
        RightEncoder.resetEncoder();
        CentralEncoder.resetEncoder();

        /*
        LP = () -> (LeftEncoder.getCurrentPosition()*(TicksPerInch));
        RP = () -> (RightEncoder.getCurrentPosition()*(TicksPerInch));
        CP = () -> (CentralEncoder.getCurrentPosition()*(TicksPerInch));
        */

        //    private Motor.Encoder LeftEncoder;
        //    private Motor.Encoder RightEncoder;
        //    private Motor.Encoder CentralEncoder;
        //HolonomicOdometry holonomicOdometry = new HolonomicOdometry(LP, RP, CP, 12.75, -8.7);
        HolonomicOdometry holonomicOdometry = new HolonomicOdometry(
        () -> (LeftEncoder.getCurrentPosition()*(TicksPerInch)),
        () -> (RightEncoder.getCurrentPosition()*(TicksPerInch)),
        () -> (CentralEncoder.getCurrentPosition()*(TicksPerInch)),
        12.75, -8.7
                );

        OdometrySubsystem odometrySubSystem = new OdometrySubsystem(holonomicOdometry);




        Waypoint startW = new StartWaypoint(odometrySubSystem.getPose().getX(), odometrySubSystem.getPose().getY());
        //Waypoint premW = new InterruptWaypoint(8192*2, 8192*2, odometry.updatePose()); //Learning "Position Buffer"
        Waypoint intermediateW = new GeneralWaypoint(0, 8192, 0, 0.8,0, 4096);
        Waypoint postW = new InterruptWaypoint();
        //Waypoint endW = new EndWaypoint(LeftEncoder.getCurrentPosition()+8192 * 11, 0, Math.PI/4, 0.6, 0.2, Math.PI, Math.PI, Math.PI );
        //Waypoint endW = new EndWaypoint(0, 8192*11, 0, 0.8, 0, 8192, 8192, 8192);
        Waypoint endW = new EndWaypoint(odometrySubSystem.getPose().getX(), Integer.MAX_VALUE, Math.PI, 0.8, 0, 5,1, 1);
        Path testP = new Path(startW,intermediateW,endW);
        //testP.setWaypointTimeouts(100);

        //testP.init();

        while(!isStopRequested()) {

            odometrySubSystem.update();

            LP = () -> LeftEncoder.getCurrentPosition();
            RP = () -> RightEncoder.getCurrentPosition();
            CP = () -> CentralEncoder.getCurrentPosition();
            holonomicOdometry.updatePose();





            TelemetryPacket TelemetryPacket = new TelemetryPacket();
            //com.acmerobotics.roadrunner.geometry.Pose2d Pose2dField= new Pose2d(OdometrySubSystem.getPose().getX(), OdometrySubSystem.getPose().getY(),OdometrySubSystem.getPose().getHeading());
            //Graph.updateConfig();
            //TelemetryPacket.put("Pure Pursuit Position Indicator", 1);
            TelemetryPacket.put("X: ", odometrySubSystem.getPose().getX());
            TelemetryPacket.put("Y: ", odometrySubSystem.getPose().getY());
            TelemetryPacket.put("H: ", odometrySubSystem.getPose().getHeading());
            telemetry.addData("X: ", LeftEncoder.getCurrentPosition());
            telemetry.addData("Y: ", RightEncoder.getCurrentPosition());
            telemetry.addData("H: ", CentralEncoder.getCurrentPosition());
            telemetry.update();
            Graph = FtcDashboard.getInstance();
            Canvas Field = TelemetryPacket.fieldOverlay();
            Pose2d Pose2dField = new Pose2d(odometrySubSystem.getPose().getX(), odometrySubSystem.getPose().getY(), odometrySubSystem.getPose().getHeading());
            DashboardUtil.drawRobot(Field, Pose2dField);
            Graph.sendTelemetryPacket(TelemetryPacket);




            if (gamepad1.a) {
                LeftEncoder.set(0);
                RightEncoder.set(0);
                CentralEncoder.set(0);
                LeftEncoder.resetEncoder();
                RightEncoder.resetEncoder();
                CentralEncoder.resetEncoder();
            }


            if (gamepad1.y && ButtonY!=1) {
                /*The Code Below Is Related to The Pure Pursuit Road Tracking Codes Above*/
                ButtonY = 1;
            }
            else if(!gamepad1.y && ButtonY == 1){

                testP.followPath(Drivers, holonomicOdometry);
                ButtonY = 0;

            }

            /*
            ControlledDrives.setWeightedDrivePower(
                    new Pose2d(

                            -0.4*gamepad1.left_stick_x,
                            -0.4*gamepad1.left_stick_y,
                            -0.3*gamepad1.right_stick_x
                    )
            );
*/

            lf.set(0.6 * (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            rf.set(0.6 * (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            lb.set(0.6 * (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            rb.set(0.6 * (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));


            if(gamepad1.x){

                stop();

            }

        }


    }

}