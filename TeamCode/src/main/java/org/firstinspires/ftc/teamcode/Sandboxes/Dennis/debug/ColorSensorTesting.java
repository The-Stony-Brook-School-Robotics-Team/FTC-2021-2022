package org.firstinspires.ftc.teamcode.Sandboxes.Dennis.debug;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name="U - Color Sensor Testing")
public class ColorSensorTesting extends OpMode {
    private Servo servo;
    private Rev2mDistanceSensor distanceSensor;
    private ColorRangeSensor blueColorRangeSensor;
    private String object;

    private boolean pUp = false, pDown = false;
    private double ejectPosition = 0; // 0.5
    private double closedPositionBall = 0.35;
    private double closedPositionCube = 0.55;
    private double readyPosition = 0.2;
    private double ballThresholdAlpha = 210;
    private double cubeThresholdRed = 68;
    private double cubeDistanceThreshold = 50;
    private double ballDistanceThreshold = 50;
    private boolean check = false;
    public boolean objectConfirmed = false;
    int sum;
    double k;
    double c;
    double m;
    double y;
    double temp;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "du");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "bbd");
        blueColorRangeSensor = hardwareMap.get(ColorRangeSensor.class, "bc");
        object = "nothing ehe :((";
        sum = blueColorRangeSensor.alpha() + blueColorRangeSensor.red() + blueColorRangeSensor.green() + blueColorRangeSensor.blue();
        servo.setPosition(readyPosition);

        temp = Math.max(blueColorRangeSensor.getNormalizedColors().red, blueColorRangeSensor.getNormalizedColors().green);
        k = Math.max(temp, blueColorRangeSensor.getNormalizedColors().blue);
        y = (1-blueColorRangeSensor.getNormalizedColors().blue-k) / (1-k);
        //servo.setPosition(closedPosition);
    }

    @Override
    public void loop() {
        temp = Math.max(blueColorRangeSensor.getNormalizedColors().red, blueColorRangeSensor.getNormalizedColors().green);
        k = Math.max(temp, blueColorRangeSensor.getNormalizedColors().blue);
        y = (1-blueColorRangeSensor.getNormalizedColors().blue-k) / (1-k);
        //sum = blueColorRangeSensor.alpha() + blueColorRangeSensor.red() + blueColorRangeSensor.green() + blueColorRangeSensor.blue();
        if(gamepad1.dpad_up && !pUp)
        {
            pUp = true;
            servo.setPosition(servo.getPosition() + 0.05);

        }
        if(pUp && !gamepad1.dpad_up)
        {
            pUp = false;
        }
        if(gamepad1.dpad_down && !pDown)
        {
            pDown = true;
            servo.setPosition(servo.getPosition() - 0.05);
        }
        if(pDown && !gamepad1.dpad_down)
        {
            pDown = false;
        }

        if(gamepad1.a){
            servo.setPosition(ejectPosition);
        }
        if(gamepad1.b){
            servo.setPosition(closedPositionCube);
        }
        if(gamepad1.y){
            servo.setPosition(readyPosition);
        }

        //if(distanceSensor.getDistance(DistanceUnit.MM) > 50){
   /**             if(blueColorRangeSensor.red() > cubeThresholdRed){
                servo.setPosition(closedPositionCube);
                object = "cube.. uwu";
            }
                else if(blueColorRangeSensor.alpha() > ballThresholdAlpha){
                    servo.setPosition(closedPositionBall);
                    object = "ball :D";

                }
                if(blueColorRangeSensor.red() < cubeThresholdRed){
                    servo.setPosition(readyPosition);
                    object = "nothing ehe :((";
                } **/
        if(blueColorRangeSensor.alpha() > 160){
            servo.setPosition(closedPositionCube);
            object = "something";
        }
        else{servo.setPosition(readyPosition); object = "nothing";}


            //else object = "defaulted to cube :(";servo.setPosition(readyPosition);
        //}



        telemetry.addData("position: ", servo.getPosition());
       // telemetry.addData("distance: ", distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("object: ", object);
      telemetry.addData("alpha: ", blueColorRangeSensor.alpha());
        //telemetry.addData("red: ", blueColorRangeSensor.red());
        //telemetry.addData("sum: ", sum);
        //telemetry.addData("normalized alpha: ", blueColorRangeSensor.getNormalizedColors().alpha);
        //telemetry.addData("normalized red: ", blueColorRangeSensor.getNormalizedColors().red);
        //telemetry.addData("normalized green: ", blueColorRangeSensor.getNormalizedColors().green);
        //telemetry.addData("normalized blue: ", blueColorRangeSensor.getNormalizedColors().blue);
        //telemetry.addData("y: ", y);

        telemetry.update();
    }
}