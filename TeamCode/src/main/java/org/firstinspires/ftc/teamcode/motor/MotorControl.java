package org.firstinspires.ftc.teamcode.motor;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * This class is used to control the motor systems on the robot.
 */
public class MotorControl {

    public final Servo claw;
    public final Servo hookArm;
    public final Slide slide;
    public final ClawArm clawArm;
    public final CRServo shooter;
    public final ColorSensor color;
    public final Servo autoPlacer;
    public final Servo seperator;
    public final DcMotor suspend;
    public final TouchSensor touch;
    public final TouchSensor magnet;

    /**
     * Gets the current state of the arm and slide together.
     *
     * @return the current state of the arm and slide system
     */
    public combinedPreset getCurrentPreset() {
        return currentPreset;
    }

    /**
     * Sets the target state of the arm and slide together.
     * @param targetPreset The new state to set the arm and slide to.
     */
    public void activatePreset(combinedPreset targetPreset) {
        currentPreset = targetPreset;
        switch (getCurrentPreset()) {
            case IDLE:
                clawArm.moveDown();
                slide.setTargetPosition(40);
                break;
            case PLACE:
                clawArm.moveDown();
                hookArm.setPosition(1);
                slide.setTargetPosition(1000);
                break;

        }
    }


    /**
     * These are the valid modes for the arm and slide system.
     */
    public enum combinedPreset {
        PLACE,
        IDLE,
        GRAB
    }

    /**
     * The current state of the arm and slide system.
     */
    private combinedPreset currentPreset;
    private combinedPreset oldMode;

    /**
     * This initializes the arm and slide motors, and resets the mode to the default. This should be run before any other methods.
     *
     * @param hardwareMap The hardware map to use to get the motors.
     */
    public MotorControl(@NonNull HardwareMap hardwareMap) {
        // TODO; probably not needed to automate init, but if you did use annotations
        clawArm = new ClawArm(hardwareMap);
        slide = new Slide(hardwareMap);


        claw = hardwareMap.get(Servo.class, "claw");
        hookArm = hardwareMap.get(Servo.class, "hookArm");
        shooter = hardwareMap.get(CRServo.class, "shooter");
        autoPlacer = hardwareMap.get(Servo.class, "autoPlacer");
        seperator = hardwareMap.get(Servo.class, "seperator");
        claw.setPosition(1);
        hookArm.setPosition(1);
        shooter.setPower(0);
        autoPlacer.setPosition(1);
        seperator.setPosition(0); // TODO tune

        suspend = hardwareMap.get(DcMotor.class, "suspend");
        color = hardwareMap.get(ColorSensor.class, "color");
        touch = hardwareMap.get(TouchSensor.class, "touch");
        magnet = hardwareMap.get(TouchSensor.class, "magnet");

        activatePreset(combinedPreset.GRAB);
    }

    /**
     * This class updates the arm and slide motors to match the current state.
     */
    public void update() {

        slide.update();
        clawArm.update();
    }


    /**
     * Reset all motors.
     */
    public void reset() {
        clawArm.reset();
        slide.reset();
    }


    public boolean closeEnough() {
        return clawArm.closeEnough() && slide.closeEnough();
    }

    public boolean isOverCurrent() {
        return clawArm.isOverCurrent() || slide.isOverCurrent();
    }

    public enum ArmPreset {
        DOWN,
        MOVING_DOWN,
        HOOK,
        MOVING_HOOK
    }
    /**
     * Lower arm control
     */
    @Config
    public class ClawArm extends ControlledMotor {
        //public static final PIDFController.PIDCoefficients PID_CONSTANTS = new PIDFController.PIDCoefficients(0.01, 0.0, 0.01);
        //public final PIDFController controller = new PIDFController(PID_CONSTANTS);

        public ArmPreset currentPreset = ArmPreset.DOWN;

        public ClawArm(@NonNull HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "arm");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(4.0, CurrentUnit.AMPS);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //controller.setOutputBounds(-0.3, 0.5);
        }

        /**
         * Stop arm and reset encoder
         */
        public void reset() {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /**
         * This updates the arm motor to match the current state. This should be run in a loop.
         */
        public void update() {
            switch (currentPreset) {
                case DOWN:
                    motor.setPower(0);
                    break;
                case MOVING_DOWN:
                    if (!touch.isPressed()) {
                        motor.setPower(-0.5);
                    } else {
                        motor.setPower(0);
                        currentPreset = ArmPreset.DOWN;
                    }
                    break;
                case HOOK:
                    motor.setPower(0);
                    break;
                case MOVING_HOOK:
                    if (!magnet.isPressed()) {
                        motor.setPower(0.5);
                    } else {
                        motor.setPower(0);
                        currentPreset = ArmPreset.HOOK;
                    }
                    break;
            }
        }

        public void movePreset(ArmPreset newPreset) {
            if (newPreset == currentPreset) {
                return;
            }
            switch (newPreset) {
                case DOWN:
                    moveDown();
                    break;
                case HOOK:
                    moveToHook();
                    break;
            }
        }

        public void moveDown() {
            currentPreset = ArmPreset.MOVING_DOWN;
        }

        public void moveToHook() {
            currentPreset = ArmPreset.MOVING_HOOK;
        }

        public boolean closeEnough() {
            return currentPreset == ArmPreset.DOWN || currentPreset == ArmPreset.HOOK;
        }


    }
    /**
     * This class controls the slide motor.
     */
    public static class Slide extends ControlledMotor {

        boolean resetting = false;
        /**
         * This initializes the slide motor. This should be run before any other methods.
         *
         * @param hardwareMap The hardware map to use to get the motors.
         */
        public Slide(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "slide");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(6, CurrentUnit.AMPS);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        /**
         * This stops the slide, sets the state to down, sets the target to 0, and resets the encoder.
         */
        public void reset() {
            motor.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        /**
         * This updates the slide motor to match the current state. This should be run in a loop.
         */
        public void update() {
            if (resetting) {
                if (motor.getCurrent(CurrentUnit.AMPS) > 2.5) {
                    reset();
                    resetting = false;
                }
            } else {
                // overly complex slide code
                // obtain the encoder position and calculate the error
                double slideError = targetPosition - motor.getCurrentPosition();
                motor.setTargetPosition((int) targetPosition);
                motor.setTargetPositionTolerance(5);
                if (slideError > 0 && !motor.isOverCurrent()) {
                    motor.setPower(0.8);
                } else {
                    motor.setPower(-0.5);
                }
                if (!motor.isOverCurrent()) {
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                }
            }
        }

        public void findZero() {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(-0.5);
            resetting = true;
        }


        public boolean closeEnough() {
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 20;
        }

    }

    /**
     * This class controls the claw.
     */

    public abstract static class ControlledMotor {
        public DcMotorEx motor;
        double targetPosition;
        public double getTargetPosition() {
            return targetPosition;
        }
        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }
        public abstract void update();
        public abstract void reset();
        public abstract boolean closeEnough();
        public boolean isOverCurrent() {
            return motor.isOverCurrent();
        }
    }
}
