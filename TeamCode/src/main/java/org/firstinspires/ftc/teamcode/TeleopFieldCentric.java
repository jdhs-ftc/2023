package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.motor.MotorControl;
import org.firstinspires.ftc.teamcode.motor.PIDFController;

import java.util.List;

@TeleOp(name = "Teleop Field Centric")
@Config
public class TeleopFieldCentric extends LinearOpMode {

    // TODO: PhotonFTC

    // Declare a PIDF Controller to regulate heading
    private final PIDFController.PIDCoefficients HEADING_PID = new PIDFController.PIDCoefficients(0.15, 0.0, 0.0);
    private final PIDFController headingController = new PIDFController(HEADING_PID);
    double speed;
    LynxModule CONTROL_HUB;
    LynxModule EXPANSION_HUB;
    boolean fieldCentric = true;
    boolean lowerClawHoldingPixel = false;
    boolean upperClawHoldingPixel = false;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    @Override
    public void runOpMode() {
        //PhotonCore.enable();

        //  Initialization Period

        // Enable Performance Optimization

        //PhotonCore.start(hardwareMap); // TODO: if somethings' wrong THIS IS WHY
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        CONTROL_HUB = allHubs.get(0);
        EXPANSION_HUB = allHubs.get(1);

        // RoadRunner Init
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        //headingController.setInputBounds(-Math.PI, Math.PI);

        // Telemetry Init
        telemetry.setMsTransmissionInterval(50);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Motor Init
        MotorControl motorControl = new MotorControl(hardwareMap);

        motorControl.setCurrentMode(MotorControl.combinedMode.IDLE);




        waitForStart();

        if (isStopRequested()) return;


        // Run Period

        while (opModeIsActive() && !isStopRequested()) {
            double prevTime = System.nanoTime(); // set to current time

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Update the speed
            if (gamepad1.left_bumper) {
                speed = .35;
            } else if (gamepad1.right_bumper) {
                speed = 1;
            } else {
                speed = .8;
            }

            if (gamepad1.dpad_left && !previousGamepad1.dpad_left) {
                if (!PoseStorage.isBlue) {
                    drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(90.0));
                } else {
                    drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(-90.0));
                }
                gamepad1.rumbleBlips(1);
            }
            if (gamepad1.share) {
                if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    fieldCentric = !fieldCentric;
                    if (fieldCentric) { gamepad1.rumbleBlips(2);}
                    else { gamepad1.rumbleBlips(1);}
                }

                if (gamepad1.dpad_left && !previousGamepad1.dpad_left) {
                    PoseStorage.isBlue = !PoseStorage.isBlue;
                    if (PoseStorage.isBlue) { gamepad1.rumbleBlips(1);}
                    else { gamepad1.rumbleBlips(2);}
                }
            }

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * speed,
                    -gamepad1.left_stick_x * speed
            );

            //Pose2d poseEstimate = drive.pose;
            double rotationAmount = -drive.pose.heading.log(); // Rotation2d.log() makes it into a double in radians.
            if (fieldCentric) {
                if (PoseStorage.isBlue) {
                    //input = drive.pose.heading.inverse().plus(90).times(new Vector2d(-input.x, input.y)); // magic courtesy of https://github.com/acmerobotics/road-runner/issues/90#issuecomment-1722674965
                    rotationAmount = rotationAmount - Math.toRadians(90);
                } else {
                    //input = drive.pose.heading.inverse().plus(-90).times(new Vector2d(input.x, -input.y)); // magic courtesy of
                    rotationAmount = rotationAmount + Math.toRadians(90);

                }
                input = new Vector2d(input.x * Math.cos(rotationAmount) - input.y * Math.sin(rotationAmount), input.x * Math.sin(rotationAmount) + input.y * Math.cos(rotationAmount));
                //input = drive.pose.heading.inverse().plus(90).times(new Vector2d(-input.x, input.y)); // magic courtesy of https://github.com/acmerobotics/road-runner/issues/90#issuecomment-1722674965
            }
            Vector2d controllerHeading = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
            if (motorControl.getCurrentMode() == MotorControl.combinedMode.PLACE) {
                input = input.plus(new Vector2d(
                        0,
                        -gamepad2.right_stick_x * 0.25
                ));
            }

            if (controllerHeading.minus(new Vector2d(0.0,0.0)).norm() < 0.7) {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        input.x,
                                        input.y
                                ),
                                (gamepad1.left_trigger - gamepad1.right_trigger) * speed
                        )
                );
            } else {
                // Set the target heading for the heading controller to our desired angle


                headingController.targetPosition = controllerHeading.angleCast().log() + Math.toRadians(180);


                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(drive.pose.heading.log())
                        * MecanumDrive.PARAMS.kV
                        * MecanumDrive.PARAMS.trackWidthTicks * MecanumDrive.PARAMS.inPerTick);
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        input.x,
                                        input.y
                                ),
                                headingInput
                        )
                );

            }
            if (gamepad1.right_stick_button) {
                headingController.targetPosition = Math.toRadians(0); // forward; aligned with pixel canvas

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(drive.pose.heading.log())
                        * MecanumDrive.PARAMS.kV)
                        * MecanumDrive.PARAMS.trackWidthTicks;
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        input.x,
                                        input.y
                                ),
                                headingInput
                        )
                );
            }



            // CLAW



            motorControl.lowerClaw.setPower(gamepad2.right_trigger);
            motorControl.upperClaw.setPower(gamepad2.left_trigger);



            if (gamepad2.right_bumper && !previousGamepad2.right_bumper) { // if bumper just pressed
                motorControl.setCurrentMode(MotorControl.combinedMode.PLACE);
            } else if (!gamepad2.right_bumper && previousGamepad2.right_bumper) {
                motorControl.setCurrentMode(MotorControl.combinedMode.IDLE); // if bumper just released
            }
            if (gamepad2.left_bumper && !previousGamepad2.left_bumper) { // if bumper just pressed
                motorControl.setCurrentMode(MotorControl.combinedMode.GRAB);
            } else if (!gamepad2.left_bumper && previousGamepad2.left_bumper) {
                motorControl.setCurrentMode(MotorControl.combinedMode.IDLE); // if bumper just released
            }


            if (gamepad2.dpad_down) {
                motorControl.arm.setTargetPosition(-10);
                motorControl.slide.setTargetPosition(-60);
            }

            // Slide
            if (motorControl.slide.getTargetPosition() >= 1000 && -gamepad2.left_stick_y > 0) {
                motorControl.slide.setTargetPosition(999);

            } else if (motorControl.slide.getTargetPosition() <= -60 && -gamepad2.left_stick_y < 0 && !gamepad2.dpad_down) {
                motorControl.slide.setTargetPosition(-60);

            } else { motorControl.slide.setTargetPosition(motorControl.slide.getTargetPosition() + (-gamepad2.left_stick_y * 40));}


            // Combined Presets
            if (gamepad2.y) {
                motorControl.setCurrentMode(MotorControl.combinedMode.PLACE);
            }
            if (gamepad2.b) {
                motorControl.setCurrentMode(MotorControl.combinedMode.IDLE);
            }
            if (gamepad2.a) {
                motorControl.setCurrentMode(MotorControl.combinedMode.GRAB);
            }

            if (gamepad2.square) {
                motorControl.shooter.setPower(0);
            } else {
                motorControl.shooter.setPower(0.5);
            }




            if (gamepad2.a && !previousGamepad2.a) {
                motorControl.arm.armController.setOutputBounds(-0.5,0.5);
            } else if (previousGamepad2.a && !gamepad2.a) {
                motorControl.arm.armController.setOutputBounds(0,0.5);
            }

            if (gamepad2.left_bumper && !previousGamepad2.left_bumper) {
                motorControl.arm.armController.setOutputBounds(-0.5,0.5);
            } else if (previousGamepad2.left_bumper && !gamepad2.left_bumper) {
                motorControl.arm.armController.setOutputBounds(0,0.5);
            }




            //gamepad1.rumble(CONTROL_HUB.getCurrent(CurrentUnit.AMPS),EXPANSION_HUB.getCurrent(CurrentUnit.AMPS),Gamepad.RUMBLE_DURATION_CONTINUOUS);
            if (motorControl.isOverCurrent()) {
                gamepad1.rumble(0.5, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                gamepad2.rumble(0.5, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            }
            drive.updatePoseEstimate();
            motorControl.update();
            // Timing
            // measure difference between current time and previous time
            double timeDifference = (System.nanoTime() - prevTime) / 1e-6;

            TelemetryPacket packet = new TelemetryPacket();
            MecanumDrive.drawRobot(packet.fieldOverlay(), drive.pose); //new Pose2d(new Vector2d(IN_PER_TICK * drive.pose.trans.x,IN_PER_TICK * drive.pose.trans.y), drive.pose.rot)
            FtcDashboard.getInstance().sendTelemetryPacket(packet);



            // Print pose to telemetry
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading.log());
            telemetry.addData("controllerHeading", controllerHeading.angleCast().log());
            telemetry.addData("loopTimeMs", timeDifference);
            telemetry.addData("loopTimeHz", 1000.0 / timeDifference);
            telemetry.addData("armTarget", motorControl.arm.getTargetPosition());
            telemetry.addData("armPosition", motorControl.arm.motor.getCurrentPosition());
            telemetry.addData("slideTarget", motorControl.slide.getTargetPosition());
            telemetry.addData("slidePosition", motorControl.slide.motor.getCurrentPosition());
            telemetry.addData("clawPower", motorControl.lowerClaw.servo.getPosition());
            telemetry.addData("leftClawHoldingPixel", lowerClawHoldingPixel);
            telemetry.addData("upperClawPower", motorControl.upperClaw.servo.getPosition());
            telemetry.addData("upperClawHoldingPixel", upperClawHoldingPixel);
            telemetry.addData("currentMode", motorControl.getCurrentMode());
            telemetry.addData("armOverCurrent", motorControl.arm.motor.isOverCurrent());
            telemetry.update();
        }
    }
}