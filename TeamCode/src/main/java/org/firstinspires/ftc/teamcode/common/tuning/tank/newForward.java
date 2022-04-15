package org.firstinspires.ftc.teamcode.common.tuning.tank;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
@Autonomous(name="new forward test")
public class newForward extends OpMode {
    SampleTankDrive drive;
    Trajectory traj;
    @Override
    public void init() {
       drive = new SampleTankDrive(hardwareMap);
       traj = new TrajectoryBuilder(drive.getPoseEstimate(), SampleTankDrive.VEL_CONSTRAINT, SampleTankDrive.accelConstraint)
               .lineTo(new Vector2d(10, 0))
               .build();
    }

    @Override
    public void start(){
       drive.followTrajectory(traj);
    }

    @Override
    public void loop() {

    }
}
