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
 * TODO: UPDATE
 */
public class MotorControl {
    public LowerClaw lowerClaw;
    public UpperClaw upperClaw;
    public Slide slide;
    public Arm arm;
    public Shooter shooter;

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
        lowerClaw = new LowerClaw(hardwareMap);
        upperClaw = new UpperClaw(hardwareMap);
        shooter = new Shooter(hardwareMap);

        setCurrentMode(combinedMode.GRAB);
    }

    /**
     * This class updates the arm and slide motors to match the current state.
     */
    public void update() {
        if (getCurrentMode() != oldMode) {
        switch (getCurrentMode()) {
            case GRAB:
                slide.setTargetPosition(-60);
                arm.moveDown();
                break;
            case IDLE:
                arm.moveDown();
                slide.setTargetPosition(70);
                break;
            case PLACE:
                arm.moveOut();
                slide.setTargetPosition(750); // TODO: TUNE
                break;

        } }
        oldMode = getCurrentMode();
        slide.update();
        arm.update();
    }


    /**
     * Reset all motors.
     */
    public void reset() {
        arm.reset();
        slide.reset();
    }


    public boolean closeEnough() {
        return arm.closeEnough() && slide.closeEnough();
    }

    public boolean isOverCurrent() {
        return arm.isOverCurrent() || slide.isOverCurrent();
    }



    /**
     * This class controls the arm motor.
     */
    @Config
    public static class Arm {
        public DcMotorEx motor;

        public static PIDFController.PIDCoefficients ARM_PID = new PIDFController.PIDCoefficients(0.05, 0.0, 0.01);
        public final PIDFController armController = new PIDFController(ARM_PID);

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
            motor.setCurrentAlert(4.4, CurrentUnit.AMPS);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armController.setOutputBounds(0,0.5);
            //motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 3, 0, 0));
        }

        /**
         * This stops the arm and resets the encoder.
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
            //double armError = Math.abs(targetPosition - motor.getCurrentPosition());
            armController.targetPosition = targetPosition;




            if (!motor.isOverCurrent()) {
                motor.setPower(armController.update(motor.getCurrentPosition()));
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 2; }

        public boolean isOverCurrent() {
            return motor.isOverCurrent();
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



        public boolean closeEnough() {
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 20; }

        public boolean isOverCurrent() {
            return motor.isOverCurrent();
        }
    }

    /**
     * This class controls the claw.
     */
    public static class LowerClaw {
        public Servo servo;

        /**
         * This initializes the claw servo. This should be run before any other methods.
         * @param hardwareMap The hardware map to use to get the servo.
         */
        public LowerClaw(HardwareMap hardwareMap) {
            servo = hardwareMap.get(Servo.class, "claw");
            servo.setPosition(0);
        }

        /**
         * This opens or closes the claw.
         * @param power The power to set the claw servo to.
         */
        public void setPower(double power) {
            servo.setPosition(power);
        }
    }
    public static class UpperClaw {
        public Servo servo;

        /**
         * This initializes the claw servo. This should be run before any other methods.
         * @param hardwareMap The hardware map to use to get the servo.
         */
        public UpperClaw(HardwareMap hardwareMap) {
            servo = hardwareMap.get(Servo.class, "upperClaw");
            servo.setPosition(0);
        }

        /**
         * This opens or closes the claw.
         * @param power The power to set the claw servo to.
         */
        public void setPower(double power) {
            servo.setPosition(power);
        }

    }
    public static class Shooter {
        public Servo servo;

        /**
         * This initializes the claw servo. This should be run before any other methods.
         * @param hardwareMap The hardware map to use to get the servo.
         */
        public Shooter(HardwareMap hardwareMap) {
            servo = hardwareMap.get(Servo.class, "shooter");
            servo.setPosition(0.5);
        }

        /**
         * This opens or closes the claw.
         * @param power The power to set the claw servo to.
         */
        public void setPower(double power) {
            servo.setPosition(power);
        }

    }

}
