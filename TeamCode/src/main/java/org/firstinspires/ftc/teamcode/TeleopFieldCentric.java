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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.motor.MotorControl;
import org.firstinspires.ftc.teamcode.motor.PIDFController;
import org.firstinspires.ftc.teamcode.vision.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.vision.pipelines.WhitePixelProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@TeleOp(name = "Teleop Field Centric")
@Config
public class TeleopFieldCentric extends LinearOpMode {

    // TODO: PhotonFTC

    // Declare a PIDF Controller to regulate heading
    private final PIDFController.PIDCoefficients HEADING_PID_JOYSTICK = new PIDFController.PIDCoefficients(0.15, 0.0, 0.0);
    private final PIDFController joystickHeadingController = new PIDFController(HEADING_PID_JOYSTICK);
    private final PIDFController.PIDCoefficients PIXEL_PID_JOYSTICK = new PIDFController.PIDCoefficients(0.002, 0.0, 0.0);
    private final PIDFController pixelHeadingController = new PIDFController(PIXEL_PID_JOYSTICK);
    double speed;
    LynxModule CONTROL_HUB;
    LynxModule EXPANSION_HUB;
    boolean fieldCentric = true;
    public enum LiftState {
        IDLE,
        PIXEL_TO_HOOK,
        HOOK_TO_BACKDROP_HOLD,
        HOOK_TO_BACKDROP_WAIT, CLAW_RELEASE, PLACE
    }
    LiftState liftState = LiftState.IDLE;
    ElapsedTime liftTimer = new ElapsedTime();
    boolean pixelInClaw = false;
    boolean pixelInHook = false;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    WhitePixelProcessor whitePixelProcessor = new WhitePixelProcessor(telemetry);
    CameraStreamProcessor cameraStreamProcessor = new CameraStreamProcessor();


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

        motorControl.activatePreset(MotorControl.combinedPreset.IDLE);


        // Vision Init
        VisionPortal myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(whitePixelProcessor, cameraStreamProcessor)
                .build();

        FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor,30);



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
                if (!(PoseStorage.currentTeam == PoseStorage.Team.BlUE)) {
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
                    if (PoseStorage.currentTeam == PoseStorage.Team.RED) {
                        gamepad1.rumbleBlips(1);
                        PoseStorage.currentTeam = PoseStorage.Team.BlUE;

                    } else {
                        gamepad1.rumbleBlips(2);
                        PoseStorage.currentTeam = PoseStorage.Team.RED;
                    }
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
                if (PoseStorage.currentTeam == PoseStorage.Team.BlUE) {
                    //input = drive.pose.heading.inverse().plus(Math.toRadians(90)).times(new Vector2d(-input.x, input.y)); // magic courtesy of https://github.com/acmerobotics/road-runner/issues/90#issuecomment-1722674965
                    rotationAmount = rotationAmount - Math.toRadians(90);
                } else {
                    //input = drive.pose.heading.inverse().plus(Math.toRadians(-90)).times(new Vector2d(input.x, -input.y)); // magic courtesy of
                    rotationAmount = rotationAmount + Math.toRadians(90);

                }
                input = new Vector2d(input.x * Math.cos(rotationAmount) - input.y * Math.sin(rotationAmount), input.x * Math.sin(rotationAmount) + input.y * Math.cos(rotationAmount));
                //input = drive.pose.heading.inverse().plus(Math.toRadians(90)).times(new Vector2d(-input.x, input.y)); // magic courtesy of https://github.com/acmerobotics/road-runner/issues/90#issuecomment-1722674965
            }
            Vector2d controllerHeading = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
            if (gamepad2.right_trigger > 0.5) {
                input = input.plus(new Vector2d(
                        gamepad2.right_stick_x * 0.5,
                        0
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


                joystickHeadingController.targetPosition = controllerHeading.angleCast().log() + Math.toRadians(180);


                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (joystickHeadingController.update(drive.pose.heading.log())
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
            if (gamepad1.right_stick_button && whitePixelProcessor.getDetectedPixel() != null) {
                pixelHeadingController.targetPosition =  300; // pixel aligned with claw

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (pixelHeadingController.update(whitePixelProcessor.getDetectedPixel().x));
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



            // LIFT



            // Slide (Manual)
            if (motorControl.slide.getTargetPosition() >= 1000 && -gamepad2.left_stick_y > 0) {
                motorControl.slide.setTargetPosition(999);

            } else if (motorControl.slide.getTargetPosition() <= -60 && -gamepad2.left_stick_y < 0 && !gamepad2.dpad_down) {
                motorControl.slide.setTargetPosition(-60);

            } else { motorControl.slide.setTargetPosition(motorControl.slide.getTargetPosition() + (-gamepad2.left_stick_y * 40));}

            switch (liftState) {
                case IDLE:
                    if (pixelInClaw && !pixelInHook && (gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5)) {
                        liftState = LiftState.PIXEL_TO_HOOK;
                    }
                    if (pixelInHook && gamepad2.right_trigger > 0.5) {
                        liftState = LiftState.HOOK_TO_BACKDROP_HOLD;
                        liftTimer.reset();
                        motorControl.clawArm.moveDown();
                        motorControl.slide.setTargetPosition(1000);
                        //motorControl.hookArm.setPosition(0.3);
                    }
                    if (pixelInHook && gamepad2.y) {
                        liftState = LiftState.HOOK_TO_BACKDROP_WAIT;
                        liftTimer.reset();
                        motorControl.clawArm.moveDown();
                        motorControl.slide.setTargetPosition(1000);
                        //motorControl.hookArm.setPosition(0.3);
                    }
                    if (pixelInHook && gamepad2.circle) {
                        liftState = LiftState.HOOK_TO_BACKDROP_WAIT;
                        liftTimer.reset();
                        motorControl.clawArm.moveDown();
                        //motorControl.hookArm.setPosition(0.3);
                    }
                    break;
                case PIXEL_TO_HOOK:
                    motorControl.clawArm.moveHook();
                    if (motorControl.clawArm.closeEnough() && Math.abs(motorControl.slide.getTargetPosition()) - 60 < 0 && motorControl.slide.closeEnough()) {
                        pixelInClaw = false;
                        liftState = LiftState.CLAW_RELEASE;
                        liftTimer.reset();
                    }
                    break;
                case CLAW_RELEASE:
                    if (liftTimer.milliseconds() > 250) {
                        pixelInHook = true;
                        motorControl.clawArm.moveDown();
                        liftState = LiftState.IDLE;
                    }
                case HOOK_TO_BACKDROP_HOLD:
                    if (liftTimer.milliseconds() > 500 && motorControl.slide.closeEnough() && !(gamepad2.right_trigger > 0.5)) {
                        motorControl.hookArm.setPosition(0.3);
                        liftState = LiftState.PLACE;
                        liftTimer.reset();
                    }
                    break;
                case HOOK_TO_BACKDROP_WAIT:
                    if (gamepad2.right_trigger > 0.5) {
                        liftState = LiftState.HOOK_TO_BACKDROP_HOLD;
                    }
                    break;
                case PLACE:
                    if (liftTimer.milliseconds() > 500) {
                        pixelInHook = false;
                        motorControl.hookArm.setPosition(1);
                        motorControl.clawArm.moveDown();
                        motorControl.slide.setTargetPosition(-40);
                        liftState = LiftState.IDLE;
                    }
                    break;
                default:
                    liftState = LiftState.IDLE;
                    break;
            }
            if (pixelInClaw) {
                motorControl.claw.setPosition(0.8);
            } else {
                motorControl.claw.setPosition(1);
            }
            if (gamepad2.right_bumper && !previousGamepad2.right_bumper) {
                pixelInClaw = !pixelInClaw;
            }

            // Reset
            if (gamepad2.dpad_down) {
                liftState = LiftState.IDLE;
                motorControl.clawArm.setTargetPosition(0);
                motorControl.slide.setTargetPosition(0);
                motorControl.hookArm.setPosition(1);
            }

            // Shooter
            if (gamepad2.square) {
                motorControl.shooter.setPower(-1);
            } else {
                motorControl.shooter.setPower(0);
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
            double colorAlpha = motorControl.color.alpha();
            double pad2rumble;


            if (colorAlpha > 10000 && (motorControl.clawArm.getTargetPosition() == 0) && motorControl.clawArm.closeEnough()) {
                pixelInClaw = true;
            }
            if (colorAlpha > 200) {
                pad2rumble = Math.log10(colorAlpha) / 6;
            } else {
                pad2rumble = 0;
            }
            gamepad2.rumble(pad2rumble, colorAlpha, Gamepad.RUMBLE_DURATION_CONTINUOUS);


            // Print pose to telemetry
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading.log());
            telemetry.addData("controllerHeading", controllerHeading.angleCast().log());
            telemetry.addData("loopTimeMs", timeDifference);
            telemetry.addData("loopTimeHz", 1000.0 / timeDifference);
            telemetry.addData("armTarget", motorControl.clawArm.getTargetPosition());
            telemetry.addData("armPosition", motorControl.clawArm.motor.getCurrentPosition());
            telemetry.addData("slideTarget", motorControl.slide.getTargetPosition());
            telemetry.addData("slidePosition", motorControl.slide.motor.getCurrentPosition());
            telemetry.addData("clawPower", motorControl.claw.getPosition());
            telemetry.addData("upperClawPower", motorControl.hookArm.getPosition());
            telemetry.addData("currentMode", motorControl.getCurrentPreset());
            telemetry.addData("stateMachineState", liftState);
            telemetry.addData("pixelInClaw", pixelInClaw);
            telemetry.addData("pixelInHook", pixelInHook);
            telemetry.addData("armOverCurrent", motorControl.clawArm.motor.isOverCurrent());
            telemetry.addData("colorAlpha", motorControl.color.alpha());
            telemetry.update();
        }
    }
}