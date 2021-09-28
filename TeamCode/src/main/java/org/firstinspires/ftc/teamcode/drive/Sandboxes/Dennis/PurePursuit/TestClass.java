package org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.PurePursuit;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="PurePursuit Testing", group="drive")
public class TestClass extends LinearOpMode {


    private static final double TRACKWIDTH = 12.75;
    private static final double CENTER_WHEEL_OFFSET = -8.7;
    private static final double WHEEL_DIAMETER = 2.0;
    private static final double TICKS_PER_REV = 8192;
    private static final double TICKS_TO_INCHES = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MotorEx lf, rf, lb, rb;
    private MecanumDrive drive;
    private OdometrySubsystem odometry;
    private MotorEx encoderLeft, encoderRight, encoderPerp;

    private PurePursuitCommand ppCommand;
    private MecanumDrive robotDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = new MotorEx(hardwareMap, "lf");
        rf = new MotorEx(hardwareMap, "leftodom");
        lb = new MotorEx(hardwareMap, "backodom");
        rb = new MotorEx(hardwareMap, "rightodom");

        encoderLeft = new MotorEx(hardwareMap, "leftodom");
        encoderRight = new MotorEx(hardwareMap, "rightodom");
        encoderPerp = new MotorEx(hardwareMap, "backodom");

        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        robotDrive = new MecanumDrive(lf, rf, lb, rb);

        HolonomicOdometry holOdom = new HolonomicOdometry(
                () -> encoderLeft.getCurrentPosition() * TICKS_TO_INCHES,
                () -> encoderRight.getCurrentPosition() * TICKS_TO_INCHES,
                () -> encoderPerp.getCurrentPosition() * TICKS_TO_INCHES,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        odometry = new OdometrySubsystem(holOdom);

        waitForStart();

        ppCommand = new PurePursuitCommand(
                robotDrive, odometry,
                new StartWaypoint(0,0),
                new GeneralWaypoint(200,0,0.8,0.8,30),
                new EndWaypoint(
                        400, 0, 0, 0.5,
                        0.5, 30, 0.8, 1
                )
        );
        ppCommand.schedule();

        while (opModeIsActive() && !isStopRequested())
        {

        }


    }


}