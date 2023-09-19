package org.firstinspires.ftc.teamcode.motor;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * This class is used to control the motor systems on the robot.
 * TODO: UPDATE
 */
public class MotorControl {
    public Claw claw;
    public Slide slide;
    public Arm arm;

    /**
     * Gets the current state of the arm and slide together.
     * @return the current state of the arm and slide system
     */
    public combinedMode getCurrentMode() {
        return currentMode;
    }

    /**
     * Sets the target state of the arm and slide together.
     * @param targetMode The new state to set the arm and slide to.
     */
    public void setCurrentMode(combinedMode targetMode) {
        currentMode = targetMode;
    }


    /**
     * These are the valid modes for the arm and slide system.
     */
    public enum combinedMode {
        PLACE,
        IDLE,
        GRAB
    }

    /**
     * The current state of the arm and slide system.
     */
    private combinedMode currentMode;
    private combinedMode oldMode;

    /**
     * This initializes the arm and slide motors, and resets the mode to the default. This should be run before any other methods.
     * @param hardwareMap The hardware map to use to get the motors.
     */
    public MotorControl(@NonNull HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
        slide = new Slide(hardwareMap);
        claw = new Claw(hardwareMap);

        setCurrentMode(combinedMode.GRAB);
    }



    /**
     * This class updates the arm and slide motors to match the current state.
     */
    public void update() {
        if (getCurrentMode() != oldMode) {
        switch (getCurrentMode()) {
            case GRAB:
                slide.setTargetPosition(0);
                arm.setTargetPosition(0);
                break;
            case IDLE:
                arm.setTargetPosition(0);
                slide.setTargetPosition(30);
                break;
            case PLACE:
                arm.setTargetPosition(72); // TODO: TUNE
                slide.setTargetPosition(1100); // TODO: TUNE
                break;

        } }
        oldMode = getCurrentMode();
        slide.update();
        arm.armUpdate();
    }



    /**
     * Reset all motors.
     */
    public void reset() {
        arm.reset();
        slide.reset();
    }

    /**
     * This class controls the arm motor.
     */
    public static class Arm {
        public DcMotorEx motor;

        public double getTargetPosition() {
            return targetPosition;
        }

        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        private double targetPosition;

        public Arm(@NonNull HardwareMap hardwareMap) {
            targetPosition = 0;
            motor = hardwareMap.get(DcMotorEx.class, "arm");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(4, CurrentUnit.AMPS);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        /**
         * This stops the arm, sets the state to down, and resets the encoder.
         */
        public void reset() {
            motor.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }



        /**
         * This updates the arm motor to match the current state. This should be run in a loop.
         */
        public void armUpdate() {
            double armError = targetPosition - motor.getCurrentPosition();
            motor.setTargetPosition((int) targetPosition);
            motor.setTargetPositionTolerance(1);
            if (armError > 0) {
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

        public void moveOut() {
            setTargetPosition(72);
        }

        public void moveDown() {
            setTargetPosition(0);
        }


        public boolean isBusy() {
            return motor.isBusy();
        }
    }

    /**
     * This class controls the slide motor.
     */
    public static class Slide {
        /**
         * This manually accesses the motor for the slide.
         */
        public DcMotorEx motor;

        public double getTargetPosition() {
            return targetPosition;
        }

        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        private double targetPosition;

        /**
         * This initializes the slide motor. This should be run before any other methods.
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


        public boolean isBusy() {
            return motor.isBusy();
        }
    }

    /**
     * This class controls the claw.
     */
    public static class Claw {
        public CRServo servo;

        /**
         * This initializes the claw servo. This should be run before any other methods.
         * @param hardwareMap The hardware map to use to get the servo.
         */
        public Claw(HardwareMap hardwareMap) {
            servo = hardwareMap.get(CRServo.class, "claw");
            servo.setPower(0.8);
        }

        /**
         * This opens or closes the claw.
         * @param power The power to set the claw servo to.
         */
        public void setPower(double power) {
            servo.setPower(power);
        }
    }
}
