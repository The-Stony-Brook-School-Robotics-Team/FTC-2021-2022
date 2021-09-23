package org.firstinspires.ftc.teamcode.Sandboxes.Michael;

import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class PPtest extends OpMode {
    private Waypoint p1;
    private Waypoint p2;


    @Override
    public void init() {
        p1 = new GeneralWaypoint(10, 10);
        p2 = new GeneralWaypoint(20, 10);

        Path path1 = new Path(p1, p2);
        path1.init();
        //double x = Math.atan2((m1-m2)/(1+(m1*m2)));

    }

    @Override
    public void loop() {

    }
}
