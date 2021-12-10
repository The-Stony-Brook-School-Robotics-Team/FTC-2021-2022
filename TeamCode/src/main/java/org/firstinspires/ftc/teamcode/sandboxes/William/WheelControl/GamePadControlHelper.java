package org.firstinspires.ftc.teamcode.sandboxes.William.WheelControl;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by William Tao on 11/4/2021.
 */
public class GamePadControlHelper {
    Gamepad gamePad;
    Runnable AProgram;
    Runnable BProgram;
    Runnable XProgram;
    Runnable YProgram;
    Runnable DpadUpProgram;
    Runnable DpadDownProgram;
    Runnable DpadLeftProgram;
    Runnable DpadRightProgram;
    Runnable StartProgram;
    Runnable BackProgram;
    Runnable GMProgram;

    private boolean isPressingA = false;
    private boolean isPressingB = false;
    private boolean isPressingX = false;
    private boolean isPressingY = false;
    private boolean isPressingDpadUp = false;
    private boolean isPressingDpadDown = false;
    private boolean isPressingDpadLeft = false;
    private boolean isPressingDpadRight = false;
    private boolean isPressingStart = false;
    private boolean isPressingBack = false;
    private boolean isPressingGM = false;

    Thread GPCHThread;

    public GamePadControlHelper(Gamepad gamePad, Runnable run) {

    }

    public void runThread() {
        GPCHThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.currentThread().isInterrupted()) {
                    checkPressingA();
                    checkPressingB();
                    checkPressingX();
                    checkPressingY();
                    checkPressingDpadUp();
                    checkPressingDpadDown();
                    checkPressingDpadLeft();
                    checkPressingDpadRight();
                    checkPressingStart();
                    checkPressingBack();
                    checkPressingGM();
                }
            }
        });
//        GPCHThread.setPriority(Thread.MIN_PRIORITY);
        GPCHThread.start();
    }

    public void stopThread() {
        GPCHThread.interrupt();
    }

    public void run() {
        checkPressingA();
        checkPressingB();
        checkPressingX();
        checkPressingY();
        checkPressingDpadUp();
        checkPressingDpadDown();
        checkPressingDpadLeft();
        checkPressingDpadRight();
        checkPressingStart();
        checkPressingBack();
        checkPressingGM();
    }

    private void checkPressingA() {
        if (gamePad.a && !isPressingA) {
            isPressingA = true;
        } else if (!gamePad.a && isPressingA) {
            AProgram.run();
            isPressingA = false;
        }
    }

    private void checkPressingB() {
        if (gamePad.b && !isPressingB) {
            isPressingB = true;
        } else if (!gamePad.b && isPressingB) {
            BProgram.run();
            isPressingB = false;
        }
    }

    private void checkPressingX() {
        if (gamePad.x && !isPressingX) {
            isPressingX = true;
        } else if (!gamePad.x && isPressingX) {
            XProgram.run();
            isPressingX = false;
        }
    }

    private void checkPressingY() {
        if (gamePad.y && !isPressingY) {
            isPressingY = true;
        } else if (!gamePad.y && isPressingY) {
            YProgram.run();
            isPressingY = false;
        }
    }

    private void checkPressingDpadUp() {
        if (gamePad.dpad_up && !isPressingDpadUp) {
            isPressingDpadUp = true;
        } else if (!gamePad.dpad_up && isPressingDpadUp) {
            DpadUpProgram.run();
            isPressingDpadUp = false;
        }
    }

    private void checkPressingDpadDown() {
        if (gamePad.dpad_down && !isPressingDpadDown) {
            isPressingDpadDown = true;
        } else if (!gamePad.dpad_down && isPressingDpadDown) {
            DpadDownProgram.run();
            isPressingDpadDown = false;
        }
    }

    private void checkPressingDpadLeft() {
        if (gamePad.dpad_left && !isPressingDpadLeft) {
            isPressingDpadLeft = true;
        } else if (!gamePad.dpad_left && isPressingDpadLeft) {
            DpadLeftProgram.run();
            isPressingDpadLeft = false;
        }
    }

    private void checkPressingDpadRight() {
        if (gamePad.dpad_right && !isPressingDpadRight) {
            isPressingDpadRight = true;
        } else if (!gamePad.dpad_right && isPressingDpadRight) {
            DpadRightProgram.run();
            isPressingDpadRight = false;
        }
    }

    private void checkPressingStart() {
        if (gamePad.start && !isPressingStart) {
            isPressingStart = true;
        } else if (!gamePad.start && isPressingStart) {
            StartProgram.run();
            isPressingStart = false;
        }
    }

    private void checkPressingBack() {
        if (gamePad.back && !isPressingBack) {
            isPressingBack = true;
        } else if (!gamePad.back && isPressingBack) {
            BackProgram.run();
            isPressingBack = false;
        }
    }

    private void checkPressingGM() {
        if (gamePad.guide && !isPressingGM) {
            isPressingGM = true;
        } else if (!gamePad.guide && isPressingGM) {
            GMProgram.run();
            isPressingGM = false;
        }
    }
}
