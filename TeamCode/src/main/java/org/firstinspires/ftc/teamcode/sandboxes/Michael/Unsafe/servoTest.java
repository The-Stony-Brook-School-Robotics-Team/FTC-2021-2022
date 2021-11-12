/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="AUGH test michael", group="Linear Opmode")
//@Disabled
public class servoTest extends LinearOpMode {
    boolean pressingUp = false;
    boolean pressingDown = false;
    boolean pressingA = false;
    boolean pressingB = false;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo servo = null;
    private Servo servo2 = null;

    @Override
    public void runOpMode() throws InterruptedException {
<<<<<<< Updated upstream:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/sandboxes/Michael/Unsafe/servoTest.java
        telemetry.addData("Status", "Initialized");
        servo = hardwareMap.get(Servo.class, "servo");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        telemetry.update();


=======
>>>>>>> Stashed changes:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Sandboxes/Michael/Unsafe/servoTest.java
        servo = hardwareMap.get(Servo.class, "servo");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        servo.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);

        double servoPos = 0;
        final double MIN = 1;
        final double MAX = .7;
        servo.setPosition(MAX);
        servo2.setPosition(MAX);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("pos1", servo.getPosition());
        telemetry.addData("pos2", servo2.getPosition());
        telemetry.update();
        waitForStart();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


<<<<<<< Updated upstream:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/sandboxes/Michael/Unsafe/servoTest.java
            //servo.setPosition(1);
            //servo2.setPosition(1);
            //Thread.sleep(500);
            telemetry.addData("Servo Position: ", servo.getPosition()*360);
            telemetry.addData("Servo2 Position: ", servo2.getPosition()*360);
            telemetry.update();
            //servo.setPosition(0.3);
            //servo2.setPosition(0.3);
            //Thread.sleep(500);

      /*      if(gamepad1.dpad_up){
                servo.setPosition(0.9);
            }
            if(gamepad1.dpad_down){
                servo.setPosition(0.3);
            } */
            servo.setPosition(0.25);
            servo2.setPosition(.25);

=======
            if(servoPos > MIN){
                servoPos = MIN;
            }
            if(servoPos < MAX ){
                servoPos = MAX;
            }
            if(gamepad1.dpad_up && !pressingUp){
                pressingUp = true;
            }
            else if(!gamepad1.dpad_up && pressingUp){
                servoPos-=.05;
                servo.setPosition(servoPos);
                servo2.setPosition(servoPos);
                pressingUp = false;
            }
            if(gamepad1.dpad_down && !pressingDown){
                pressingDown = true;
            }
            else if(!gamepad1.dpad_down && pressingDown){
                servoPos+=.05;
                servo.setPosition(servoPos);
                servo2.setPosition(servoPos);
                pressingDown = false;
            }
            if(gamepad1.a && !pressingA){
                pressingA = true;
            }
            else if(!gamepad1.a && pressingA){
                servo.setPosition(MIN);
                servo2.setPosition(MIN);
                pressingA = false;
            }
            if(gamepad1.b && !pressingB){
                pressingB = true;
            }
            else if(!gamepad1.b && pressingB){
                servo.setPosition(MAX);
                servo2.setPosition(MAX);
                pressingB = false;
            }


            telemetry.addData("Servo Position: ", servo.getPosition());
            telemetry.addData("Servo2 Position: ", servo2.getPosition());
            telemetry.update();
>>>>>>> Stashed changes:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Sandboxes/Michael/Unsafe/servoTest.java







            /**if(gamepad1.a && pressingA == false){
                pressingA = true;
            }
            else if(!gamepad1.a && pressingA == true){
                servo.setPosition(0);
                servo.setDirection(Servo.Direction.REVERSE);
            }**/
        }
    }
}
