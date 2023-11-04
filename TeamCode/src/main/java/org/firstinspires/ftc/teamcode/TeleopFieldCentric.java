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
    private final PIDFController.PIDCoefficients HEADING_PID_JOYSTICK = new PIDFController.PIDCoefficients(0.2, 0.0, 1);
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
    ElapsedTime loopTime = new ElapsedTime();
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
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        CONTROL_HUB = allHubs.get(0);
        EXPANSION_HUB = allHubs.get(1);

        // RoadRunner Init
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        joystickHeadingController.setInputBounds(-Math.PI, Math.PI);

        // Telemetry Init
        telemetry.setMsTransmissionInterval(50);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Motor Init
        MotorControl motorControl = new MotorControl(hardwareMap);

        motorControl.activatePreset(MotorControl.combinedPreset.IDLE);


        // Vision Init
        new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(whitePixelProcessor, cameraStreamProcessor)
                .build();

        FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor,30);



        waitForStart();

        if (isStopRequested()) return;


        // Run Period

        while (opModeIsActive() && !isStopRequested()) {
            loopTime.reset();
            allHubs.forEach(LynxModule::clearBulkCache);

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // CONTROLS

            // Gamepad 1
            // Driving Modifiers
            boolean padSlowMode = gamepad1.left_bumper;
            boolean padFastMode = gamepad1.right_bumper;
            boolean padResetPose = gamepad1.dpad_left && !previousGamepad1.dpad_left;

            // Misc/Obscure
            boolean padCameraAutoAim = gamepad1.right_stick_button;

            // Extra Settings
            boolean pad1ExtraSettings = gamepad1.share;
            boolean pad1ExTeamSwitch = gamepad1.dpad_left && !previousGamepad1.dpad_left; // 1 rumble blue, 2 rumble red
            boolean pad1ExToggleFieldCentric = gamepad1.dpad_up && !previousGamepad1.dpad_up;




            // Gamepad 2
            // Presets/Automated
            boolean padHalfCycle = gamepad2.left_trigger > 0.25;
            boolean padFullCycle = gamepad2.right_trigger > 0.25;

            boolean padHighPreset = gamepad2.y;
            boolean padMidPreset = gamepad2.b;
            boolean padLowPreset = gamepad2.a;

            boolean padClawToggle = (gamepad2.right_bumper && !previousGamepad2.right_bumper) || (gamepad1.square && !previousGamepad1.square);

            boolean padShooter = gamepad2.square;

            // Manual Control
            double padSlideControl = -gamepad2.left_stick_y;
            double padSlideControlMultiplier = 40;

            double padArmControl = -gamepad2.right_stick_y;
            double padArmControlMultiplier = 2;

            // Misc
            double padGunnerDrive = gamepad2.right_stick_x; // only when right trigger held
            boolean padForceDown = gamepad2.dpad_down;
            boolean padMissedHook = gamepad2.dpad_up;




            // Update the speed
            if (padSlowMode) {
                speed = .35;
            } else if (padFastMode) {
                speed = 1;
            } else {
                speed = .8;
            }

            if (padResetPose) {
                if (!(PoseStorage.currentTeam == PoseStorage.Team.BlUE)) {
                    drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(90.0));
                } else {
                    drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(-90.0));
                }
                gamepad1.rumbleBlips(1);
            }
            if (pad1ExtraSettings) {
                if (pad1ExToggleFieldCentric) {
                    fieldCentric = !fieldCentric;
                    if (fieldCentric) { gamepad1.rumbleBlips(2);}
                    else { gamepad1.rumbleBlips(1);}
                }

                if (pad1ExTeamSwitch) {
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
            if (fieldCentric && !padCameraAutoAim) {
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
            if (padFullCycle) {
                input = input.plus(new Vector2d(
                        0,
                        -padGunnerDrive * 0.5
                ));
            }

            if (Math.sqrt(Math.pow(controllerHeading.x, 2.0) + Math.pow(controllerHeading.y, 2.0)) < 0.4) {
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

                if (PoseStorage.currentTeam == PoseStorage.Team.BlUE) {
                    joystickHeadingController.targetPosition = controllerHeading.angleCast().log() + Math.toRadians(-90);
                } else {
                    joystickHeadingController.targetPosition = controllerHeading.angleCast().log() + Math.toRadians(90);
                }


                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (joystickHeadingController.update(drive.pose.heading.log())
                        * MecanumDrive.PARAMS.kV
                        * MecanumDrive.PARAMS.trackWidthTicks); // TODO: tune
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
            if (padCameraAutoAim && whitePixelProcessor.getDetectedPixel() != null) {
                double x = whitePixelProcessor.getDetectedPixel().x;
                double y = whitePixelProcessor.getDetectedPixel().y;
                pixelHeadingController.targetPosition =  360; // pixel aligned with claw

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (pixelHeadingController.update(x) * (y / 320));
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
            if (motorControl.slide.getTargetPosition() > 1100 && padSlideControl > 0) {
                motorControl.slide.setTargetPosition(1100);

            } else if (motorControl.slide.getTargetPosition() <= -60 && padSlideControl < 0 && !padForceDown) {
                motorControl.slide.setTargetPosition(-59);

            } else { motorControl.slide.setTargetPosition(motorControl.slide.getTargetPosition() + (padSlideControl * padSlideControlMultiplier));}

            // Arm (Manual)
            // Slide (Manual)
            if (motorControl.clawArm.getTargetPosition() > 130 && padArmControl > 0) {
                motorControl.clawArm.setTargetPosition(130);

            } else if (motorControl.clawArm.getTargetPosition() <= 0 - (padArmControl * padArmControlMultiplier)  && padArmControl < 0 && !padForceDown) {
                motorControl.clawArm.setTargetPosition(0.1);

            } else { motorControl.clawArm.setTargetPosition(motorControl.clawArm.getTargetPosition() + (padArmControl * padArmControlMultiplier));}

            switch (liftState) {
                case IDLE:
                    if (pixelInClaw && !pixelInHook && (padHalfCycle || padFullCycle) && motorControl.slide.motor.getCurrentPosition() < 850) {
                        liftState = LiftState.PIXEL_TO_HOOK;
                        liftTimer.reset();
                    }
                    if (pixelInHook && padFullCycle) {
                        liftState = LiftState.HOOK_TO_BACKDROP_HOLD;
                        liftTimer.reset();
                        motorControl.clawArm.moveDown();
                        motorControl.slide.setTargetPosition(1000);
                        //motorControl.hookArm.setPosition(0.3);
                    }
                    if (pixelInHook && padHighPreset) {
                        liftState = LiftState.HOOK_TO_BACKDROP_WAIT;
                        liftTimer.reset();
                        motorControl.clawArm.moveDown();
                        motorControl.slide.setTargetPosition(1000);
                        //motorControl.hookArm.setPosition(0.3);
                    }
                    if (pixelInHook && padMidPreset) {
                        liftState = LiftState.HOOK_TO_BACKDROP_WAIT;
                        liftTimer.reset();
                        motorControl.clawArm.moveDown();
                        //motorControl.hookArm.setPosition(0.3);
                    }
                    break;
                case PIXEL_TO_HOOK:
                    motorControl.clawArm.moveToHook();
                    motorControl.slide.setTargetPosition(0);
                    if (liftTimer.milliseconds() > 750) {
                        pixelInClaw = false;
                        pixelInHook = true;
                        liftState = LiftState.CLAW_RELEASE;
                        liftTimer.reset();
                    }
                    break;
                case CLAW_RELEASE:
                    if (liftTimer.milliseconds() > 250) {
                        motorControl.clawArm.moveDown();
                        liftState = LiftState.IDLE;
                    }
                    break;
                case HOOK_TO_BACKDROP_HOLD:
                    if (liftTimer.milliseconds() > 200 && motorControl.slide.closeEnough() && !padFullCycle) {
                        motorControl.hookArm.setPosition(0.3);
                        liftState = LiftState.PLACE;
                        liftTimer.reset();
                    }
                    break;
                case HOOK_TO_BACKDROP_WAIT:
                    if (padFullCycle) {
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
                motorControl.claw.setPosition(0.94);
            }

            if (padClawToggle) {
                pixelInClaw = !pixelInClaw;
            }
            if (padMissedHook) {
                pixelInHook = false;
            }




            // Reset
            if (padForceDown) {
                liftState = LiftState.IDLE;
                motorControl.clawArm.setTargetPosition(0);
                motorControl.slide.setTargetPosition(0);
                motorControl.hookArm.setPosition(1);
            }

            // Shooter
            if (padShooter) {
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


            TelemetryPacket packet = new TelemetryPacket();
            MecanumDrive.drawRobot(packet.fieldOverlay(), drive.pose); //new Pose2d(new Vector2d(IN_PER_TICK * drive.pose.trans.x,IN_PER_TICK * drive.pose.trans.y), drive.pose.rot)
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            double colorAlpha = motorControl.color.alpha();
            double pad2rumble;

            /*
            if (colorAlpha > 10000 && (motorControl.clawArm.getTargetPosition() == 0) && motorControl.clawArm.closeEnough() && !gamepad2.left_bumper && !gamepad1.square) {
                pixelInClaw = true;
            }

             */
            if (colorAlpha > 200 && !pixelInClaw) {
                pad2rumble = Math.log10(colorAlpha) / 6;
            } else {
                pad2rumble = 0;
            }
            gamepad2.rumble(pad2rumble, pad2rumble, Gamepad.RUMBLE_DURATION_CONTINUOUS);

            double loopTimeMs = loopTime.milliseconds();
            // Print pose to telemetry
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading.log());
            telemetry.addData("controllerHeading", controllerHeading.angleCast().log());
            telemetry.addData("loopTimeMs", loopTimeMs);
            telemetry.addData("loopTimeHz", 1000.0 / loopTimeMs);
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
            telemetry.addData("colorAlpha", colorAlpha);
            telemetry.addData("colorAlphaLog", Math.log10(colorAlpha));
            telemetry.addData("elapsedTime", liftTimer.milliseconds());
            telemetry.update();
        }
    }
}