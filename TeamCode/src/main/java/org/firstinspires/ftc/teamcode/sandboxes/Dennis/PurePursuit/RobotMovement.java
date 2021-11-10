package org.firstinspires.ftc.teamcode.sandboxes.Dennis.PurePursuit;

import static org.firstinspires.ftc.teamcode.sandboxes.Dennis.PurePursuit.math.MathFunctions.AngleWrap;

import com.qualcomm.robotcore.util.Range;

public class RobotMovement {

    /**
     *
     * @param x
     * @param y
     * @param movementSpeed
     */
    public static void goToPosition(double x, double y, double movementSpeed, double prefferedAngle, double turnSpeed) {

        double distanceToTarget = Math.hypot(x - Robot.currentWorldPosition.getX(), y - Robot.currentWorldPosition.getY());

        double absoluteAngleToTarget = Math.atan2(
                y - Robot.currentWorldPosition.getY(),
                x - Robot.currentWorldPosition.getX());

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (Math.toRadians(Robot.currentWorldPosition.getHeading()) - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        Robot.MovementX = movementXPower * movementSpeed;
        Robot.MovementY = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + prefferedAngle;
        Robot.MovementTurn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        if(distanceToTarget < 10 ) {
            Robot.MovementTurn = 0;

        }

    }
}
