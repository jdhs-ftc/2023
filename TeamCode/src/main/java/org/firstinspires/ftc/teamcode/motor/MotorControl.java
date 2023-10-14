package org.firstinspires.ftc.teamcode.motor;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * This class is used to control the motor systems on the robot.
 */
public class MotorControl {
    public Servo lowerClaw;
    public Servo upperClaw;
    public Slide slide;
    public ClawArm clawArm;
    public HookArm hookArm;
    public Servo shooter;

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
        clawArm = new ClawArm(hardwareMap);
        hookArm = new HookArm(hardwareMap);
        slide = new Slide(hardwareMap);


        lowerClaw = hardwareMap.get(Servo.class, "claw");
        upperClaw = hardwareMap.get(Servo.class, "upperClaw");
        shooter = hardwareMap.get(Servo.class, "shooter");
        lowerClaw.setPosition(0);
        upperClaw.setPosition(0);
        shooter.setPosition(0.5); // todo; change hardwaremap

        activatePreset(combinedPreset.GRAB);
    }

    /**
     * This class updates the arm and slide motors to match the current state.
     */
    public void update() {
        if (getCurrentPreset() != oldMode) {
            switch (getCurrentPreset()) {
                case GRAB:
                    slide.setTargetPosition(-40);
                    clawArm.moveDown();
                    break;
                case IDLE:
                    clawArm.moveDown();
                    slide.setTargetPosition(-40);
                    break;
                case PLACE:
                    clawArm.moveOut();
                    slide.setTargetPosition(750);
                    break;

            }
        }
        oldMode = getCurrentPreset();
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


    /**
     * Lower arm control
     */
    @Config
    public static class ClawArm extends ControlledMotor {
        public static PIDFController.PIDCoefficients PID_CONSTANTS = new PIDFController.PIDCoefficients(0.05, 0.0, 0.01);
        public final PIDFController controller = new PIDFController(PID_CONSTANTS);

        public double getTargetPosition() {
            return targetPosition;
        }

        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        private double targetPosition;

        public ClawArm(@NonNull HardwareMap hardwareMap) {
            targetPosition = 0;
            motor = hardwareMap.get(DcMotorEx.class, "arm");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(4.4, CurrentUnit.AMPS);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            controller.setOutputBounds(-0.5, 0.5);
        }

        /**
         * Stop arm and reset encoder
         */
        public void reset() {
            motor.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        /**
         * This updates the arm motor to match the current state. This should be run in a loop.
         */
        public void update() {
            controller.targetPosition = targetPosition;
            if (!motor.isOverCurrent()) {
                motor.setPower(controller.update(motor.getCurrentPosition()));
            } else {
                motor.setPower(0);

            }
        }

        public void moveOut() {
            setTargetPosition(60);
        }

        public void moveDown() {
            setTargetPosition(0);
        }

        public void moveTop() {
            setTargetPosition(120);
        }

        public boolean closeEnough() {
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 2;
        }

        public boolean isOverCurrent() {
            return motor.isOverCurrent();
        }
    }

    /**
     * Lower arm control
     */
    @Config
    public static class HookArm extends ControlledMotor{
        public static PIDFController.PIDCoefficients ARM_PID = new PIDFController.PIDCoefficients(0.05, 0.0, 0.01);
        public final PIDFController armController = new PIDFController(ARM_PID);

        public double getTargetPosition() {
            return targetPosition;
        }

        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        private double targetPosition = 0;

        public HookArm(@NonNull HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "hookArm"); //TODO: set hardwaremap
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(4.4, CurrentUnit.AMPS);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armController.setOutputBounds(-0.5, 0.5);
        }

        /**
         * Stop arm and reset encoder
         */
        public void reset() {
            motor.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        /**
         * This updates the arm motor to match the current state. This should be run in a loop.
         */
        public void update() {
            armController.targetPosition = targetPosition;
            if (!motor.isOverCurrent()) {
                motor.setPower(armController.update(motor.getCurrentPosition()));
            } else {
                motor.setPower(0);

            }
        }

        public void moveOut() {
            setTargetPosition(60);
        }

        public void moveDown() {
            setTargetPosition(0);
        }

        public void moveTop() {
            setTargetPosition(120);
        }

        public boolean closeEnough() {
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 2;
        }

        public boolean isOverCurrent() {
            return motor.isOverCurrent();
        }
    }

    /**
     * This class controls the slide motor.
     */
    public static class Slide extends ControlledMotor {
        public double getTargetPosition() {
            return targetPosition;
        }

        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        private double targetPosition;

        /**
         * This initializes the slide motor. This should be run before any other methods.
         *
         * @param hardwareMap The hardware map to use to get the motors.
         */
        public Slide(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "slide");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(8, CurrentUnit.AMPS);
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
            // overly complex slide code
            // obtain the encoder position and calculate the error
            double slideError = targetPosition - motor.getCurrentPosition();
            motor.setTargetPosition((int) targetPosition);
            motor.setTargetPositionTolerance(10);
            if (slideError > 0) {
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


        public boolean closeEnough() {
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 20;
        }

        public boolean isOverCurrent() {
            return motor.isOverCurrent();
        }
    }

    /**
     * This class controls the claw.
     */

    public abstract static class ControlledMotor {
        public DcMotorEx motor;
        abstract void update();
        abstract void reset();
        abstract boolean closeEnough();
        abstract boolean isOverCurrent();
    }
}
