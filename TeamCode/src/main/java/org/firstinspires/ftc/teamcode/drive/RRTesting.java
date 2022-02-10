package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.ParametricCurve;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionSegment;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.tensorflow.lite.task.text.qa.QaAnswer;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class RRTesting extends LinearOpMode {
    private boolean qA;





    @Override
    public void runOpMode() throws InterruptedException {
        RoadRunnerController RRctrl = new RoadRunnerController(hardwareMap,telemetry);

        waitForStart();

        while(opModeIsActive() && !isStopRequested())
        {
            Pose2d current = RRctrl.getPos();
            Pose2d vels = RRctrl.getDrive().getPoseVelocity();
            try {
            telemetry.addData("current x",current.getX());
            telemetry.addData("current y",current.getY());
            telemetry.addData("vel x",vels.getX());
            telemetry.addData("vel y",vels.getY());}
            catch (Exception e)
            {
                e.printStackTrace();
            }
            finally {
                telemetry.update();
            }
            //telemetry.addData("Trajectory Info",RRctrl.getDrive().trajectorySequenceRunner.currentTrajectorySequence);
            RRctrl.getDrive().update();
            RRctrl.getDrive().setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            RRctrl.getDrive().update();
            if(gamepad1.a && !qA)
            {
                qA = true;
                /*double iniTime = NanoClock.system().seconds();
                RRctrl.getDrive().followTrajectoryTime(
                        RRctrl.getDrive().trajectoryBuilder(new Pose2d(RRctrl.getPos().getX()+15,RRctrl.getPos().getY(),RRctrl.getPos().getHeading()))
                                .forward(40)
                                .build(),
                        iniTime
                );*/

                current = RRctrl.getPos();
                vels = RRctrl.getDrive().getPoseVelocity();
                // Create a spoofed trajectory
                Trajectory spoofed = createSpoofedTraj(current,vels,new Vector2d(4,0),0.7);
                Trajectory tester = RRctrl.getDrive().trajectoryBuilder(new Pose2d(current.getX()+4,current.getY(),current.getHeading()))
                        .splineToSplineHeading(new Pose2d(current.getX()+20,current.getY(),current.getHeading()),Math.PI/2) // this should now be smooth!!!
                        .build();
                // Chain the spoofed trajectory and the planned one, together
                TrajectorySequence trajSeq = RRctrl.getDrive().trajectorySequenceBuilder(current)
                        .addTrajectory(spoofed)
                        .addTrajectory(tester)
                        .build();

                // Follow the trajectory sequence
                double v0 = Math.sqrt(vels.getX()*vels.getX() + vels.getY()*vels.getY()); // ini vel mag
               // TrajectoryBuilder engine = new TrajectoryBuilder(current, 0,spoofed,0,null,null,new MotionState(0,v0,0),0);
                /*
                *  startPose: Pose2d?,
                    startTangent: Double?,
                    trajectory: Trajectory?,
                    t: Double?,
                    private val baseVelConstraint: TrajectoryVelocityConstraint,
                    private val baseAccelConstraint: TrajectoryAccelerationConstraint,
                    private val start: MotionState,
                    private val resolution: Double
                */
                RRctrl.getDrive().followTrajectorySequence(trajSeq);

                // lets see!!
            }
            if(!gamepad1.a && qA)
            {
                qA = false;
            }
        }
    }
    public Trajectory createSpoofedTraj(Pose2d inipos, Pose2d inivel, Vector2d dPos, double dt) {
        double x = inipos.getX(); // current pos
        double y = inipos.getY(); // current pos
        double h = inipos.getHeading(); // current pos
        double dx = dPos.getX(); // delta
        double dy = dPos.getY(); // delta
        double v0 = Math.sqrt(inivel.getX()*inivel.getX() + inivel.getY()*inivel.getY()); // ini vel mag


        // Create a motion profile segment "spoofed"
        MotionState state = new MotionState(x,v0,0,0);
        MotionSegment motionSegment = new MotionSegment(state,dt);
        // Make a new segment list and add the spoofed segment
        List<MotionSegment> motionSegs = new ArrayList<>(1);
        motionSegs.add(motionSegment);
        // Create a motion profile based off of the motion segments
        MotionProfile motionProfile = new MotionProfile(motionSegs);

        // Create the curve to follow
        ParametricCurve curve = new LineSegment(new Vector2d(x,y),new Vector2d(x+dx,y+dy));
        // Make a path segment based off of the curve and add it to a path segment list
        PathSegment pathSegment = new PathSegment(curve);
        List<PathSegment> pathSegs = new ArrayList<>(1);
        pathSegs.add(pathSegment);

        // Create a path based off of the path segment list
        Path path = new Path(pathSegs);

        // Build the spoofed trajectory
        Trajectory spoofed = new Trajectory(path,motionProfile);
        return spoofed;
    }
}
