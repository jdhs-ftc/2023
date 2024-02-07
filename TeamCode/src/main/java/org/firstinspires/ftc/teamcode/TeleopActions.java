package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController;
import org.firstinspires.ftc.teamcode.helpers.vision.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.motor.MotorActions;
import org.firstinspires.ftc.teamcode.motor.MotorControl;
import org.firstinspires.ftc.teamcode.vision.pipelines.WhitePixelProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;
@Photon
@TeleOp(name = "Teleop Field Centric")
@Config
public class TeleopActions extends ActionOpMode {


    // Declare a PIDF Controller to regulate heading
    private final PIDFController.PIDCoefficients HEADING_PID_JOYSTICK = new PIDFController.PIDCoefficients(0.2, 0.0, 1);
    private final PIDFController joystickHeadingController = new PIDFController(HEADING_PID_JOYSTICK);
    private final PIDFController.PIDCoefficients PIXEL_PID_JOYSTICK = new PIDFController.PIDCoefficients(0.004, 0.0, 0.0);
    private final PIDFController pixelHeadingController = new PIDFController(PIXEL_PID_JOYSTICK);
    double speed;
    LynxModule CONTROL_HUB;
    LynxModule EXPANSION_HUB;
    boolean fieldCentric = true;
    public MecanumDrive drive;

    List<Action> runningActions = new ArrayList<>();


    public enum LiftState {
        IDLE,
        PIXEL_TO_HOOK,
        HOOK_TO_BACKDROP_HOLD,
        HOOK_TO_BACKDROP_WAIT, CLAW_RELEASE, PLACE
    }

    LiftState liftState = LiftState.IDLE;
    final ElapsedTime liftTimer = new ElapsedTime();
    final ElapsedTime loopTime = new ElapsedTime();
    boolean pixelInClaw = false;
    boolean pixelInHook = true;//false;
    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();
    final WhitePixelProcessor whitePixelProcessor = new WhitePixelProcessor(telemetry);
    final CameraStreamProcessor cameraStreamProcessor = new CameraStreamProcessor();

    boolean showMotorTelemetry = true;
    boolean showStateTelemetry = true;
    boolean showLoopTimes = true;
    boolean showTelemetryMenu = false;
    boolean showPoseTelemetry = true;
    boolean showCameraTelemetry = false;

    MotorControl motorControl;
    MotorActions motorActions;


    @Override
    public void runOpMode() {
        //PhotonCore.enable();

        //  Initialization Period

        // Enable Performance Optimization

        //PhotonCore.start(hardwareMap); // TODO: if somethings' wrong THIS IS WHY

        // Enable Bulk Caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        CONTROL_HUB = allHubs.get(0);
        EXPANSION_HUB = allHubs.get(1);

        // RoadRunner Init
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        joystickHeadingController.setInputBounds(-Math.PI, Math.PI);

        // Telemetry Init
        telemetry.setMsTransmissionInterval(50);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Motor Init
        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);

        motorControl.slide.findZero();
        motorControl.activatePreset(MotorControl.combinedPreset.IDLE);


        // Vision Init
        new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessors(whitePixelProcessor, cameraStreamProcessor)
                .build();

        FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor, 30);


        waitForStart();

        if (isStopRequested()) return;


        // Run Period

        while (opModeIsActive() && !isStopRequested()) {
            // Reset measured loop time
            loopTime.reset();
            // Reset bulk cache
            allHubs.forEach(LynxModule::clearBulkCache);

            // This lets us do reliable rising edge detection, even if it changes mid loop
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
            boolean padFullCycle = gamepad2.right_trigger > 0.25 || gamepad1.circle;

            boolean padHighPreset = gamepad2.y;
            boolean padMidPreset = gamepad2.b;
            boolean padLowPreset = gamepad2.a;

            boolean padClawToggle = (gamepad2.right_bumper && !previousGamepad2.right_bumper); //|| (gamepad1.square && !previousGamepad1.square);

            boolean padShooter = gamepad2.square; //|| gamepad1.square;

            // Manual Control
            double padSlideControl = -gamepad2.left_stick_y;
            double padSlideControlMultiplier = 40;
            /*
            double padArmControl = -gamepad2.right_stick_y;
            double padArmControlMultiplier = 2;

             */

            double padSuspendControl = -gamepad2.right_stick_y;
            double padSuspendControlMultiplier = 1;


            // Misc
            double padGunnerDrive = gamepad2.right_stick_x; // only when right trigger held
            boolean padForceDown = false;//gamepad2.dpad_down;
            boolean padMissedHook = gamepad2.dpad_up;


            // Update the speed
            if (padSlowMode) {
                speed = .35;
            } else if (padFastMode) {
                speed = 1.5;
            } else {
                speed = .8; // prev 0.8
            }
            // especially in driver practice, imu drifts eventually
            // this lets them reset just in case
            if (padResetPose) {
                if (!(PoseStorage.currentTeam == PoseStorage.Team.BLUE)) { // Team is declared and saved there for auto
                    drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(90.0));
                } else {
                    drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(-90.0));
                }
                gamepad1.rumbleBlips(1); // tell the driver it succeeded
            }
            // Second layer
            if (pad1ExtraSettings) {
                if (pad1ExToggleFieldCentric) {
                    fieldCentric = !fieldCentric;
                    if (fieldCentric) {
                        gamepad1.rumbleBlips(2);
                    } else {
                        gamepad1.rumbleBlips(1);
                    }
                }

                if (pad1ExTeamSwitch) {
                    if (PoseStorage.currentTeam == PoseStorage.Team.RED) {
                        gamepad1.rumbleBlips(1);
                        PoseStorage.currentTeam = PoseStorage.Team.BLUE;

                    } else {
                        gamepad1.rumbleBlips(2);
                        PoseStorage.currentTeam = PoseStorage.Team.RED;
                    }
                }
            }

            // Field Centric

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * speed,
                    -gamepad1.left_stick_x * speed
            );

            //Pose2d poseEstimate = drive.pose;
            double rotationAmount = -drive.pose.heading.log(); // Rotation2d.log() makes it into a double in radians.
            if (fieldCentric && !padCameraAutoAim) {
                if (PoseStorage.currentTeam == PoseStorage.Team.BLUE) { // Depending on which side we're on the color changes
                    //input = drive.pose.heading.inverse().plus(Math.toRadians(90)).times(new Vector2d(-input.x, input.y)); // magic courtesy of https://github.com/acmerobotics/road-runner/issues/90#issuecomment-1722674965
                    rotationAmount = rotationAmount - Math.toRadians(90);
                } else {
                    //input = drive.pose.heading.inverse().plus(Math.toRadians(-90)).times(new Vector2d(input.x, -input.y)); // magic courtesy of
                    rotationAmount = rotationAmount + Math.toRadians(90);

                }
                input = Rotation2d.fromDouble(rotationAmount).times(new Vector2d(input.x, input.y)); // magic courtesy of https://github.com/acmerobotics/road-runner/issues/90#issuecomment-1722674965
            }
            Vector2d controllerHeading = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);

            // gunner can take control temporarily if needed
            if (padFullCycle) {
                input = input.plus(new Vector2d(
                        0,
                        -padGunnerDrive * 0.5
                ));
            }
            if (!driveActionRunning()) {
                // check if the right stick is pushed to the edge
                // this if statement is backwards; if true, it's not pushed drive normal
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
                } else { // right stick is pushed
                    // Set the target heading for the heading controller to our desired angle

                    // Cast the angle based on the angleCast of the joystick as a heading
                    if (PoseStorage.currentTeam == PoseStorage.Team.BLUE) {
                        joystickHeadingController.targetPosition = controllerHeading.angleCast().log() + Math.toRadians(-90);
                    } else {
                        joystickHeadingController.targetPosition = controllerHeading.angleCast().log() + Math.toRadians(90);
                    }


                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput = (joystickHeadingController.update(drive.pose.heading.log())
                            * MecanumDrive.PARAMS.kV
                            * MecanumDrive.PARAMS.trackWidthTicks);
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
                // auto aim stuff; not really tested
                if (padCameraAutoAim && whitePixelProcessor.getDetectedPixel() != null) {
                    double x = whitePixelProcessor.getDetectedPixel().x;
                    double y = whitePixelProcessor.getDetectedPixel().y;
                    pixelHeadingController.targetPosition = 200; // pixel aligned with claw

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput = (pixelHeadingController.update(x) * (y / 320));
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(
                                            input.x,
                                            headingInput
                                    ),
                                    (gamepad1.left_trigger - gamepad1.right_trigger) * speed
                            )
                    );
                }
            }


            // LIFT CONTROL/FSM


            // Slide (Manual)
            if (motorControl.slide.getTargetPosition() > 1100 && padSlideControl > 0) {
                motorControl.slide.setTargetPosition(1100);

            } else if (motorControl.slide.getTargetPosition() <= 40 && padSlideControl < 0 && !padForceDown) {
                motorControl.slide.setTargetPosition(40);

            } else {
                motorControl.slide.setTargetPosition(motorControl.slide.getTargetPosition() + (padSlideControl * padSlideControlMultiplier));
            }

            // Arm (Manual)
            // Slide (Manual)
            /*
            if (motorControl.clawArm.getTargetPosition() > 130 && padArmControl > 0) {
                motorControl.clawArm.setTargetPosition(130);

            } else if (motorControl.clawArm.getTargetPosition() <= 0 - (padArmControl * padArmControlMultiplier)  && padArmControl < 0 && !padForceDown) {
                motorControl.clawArm.setTargetPosition(0.1);

            } else { motorControl.clawArm.setTargetPosition(motorControl.clawArm.getTargetPosition() + (padArmControl * padArmControlMultiplier));}

             */

            motorControl.suspend.setPower(padSuspendControl * padSuspendControlMultiplier);

            // THE FULL STATE MACHINE

            if (pixelInClaw && padHalfCycle && motorControl.slide.motor.getCurrentPosition() < 850 && runningActions.isEmpty()) {
                run(pixelToHook());
            }
            if (pixelInHook && padFullCycle && runningActions.isEmpty()) {
                run(placePixel(this::padRelease)); // TODO: maybe causes issues?
            }

            /*
            if (pixelInClaw && runningActions.isEmpty()) {
                motorControl.claw.setPosition(0.82); //0.85
            } else {
                motorControl.claw.setPosition(0.94);
            }

             */

            if (padClawToggle) {
                pixelInClaw = !pixelInClaw;
                if (pixelInClaw) {
                    motorControl.claw.setPosition(0.82); //0.85
                } else {
                    motorControl.claw.setPosition(0.94);
                }
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

            /*
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                runningActions.add(motorActions.seperator.hold());
            } else if (!currentGamepad2.dpad_down && previousGamepad2.dpad_down){
                runningActions.add(motorActions.seperator.release());
            }

             */

            if (gamepad2.dpad_down) {
                motorControl.seperator.setPosition(0.35);
            } else {
                motorControl.seperator.setPosition(0);
            }

            //gamepad1.rumble(CONTROL_HUB.getCurrent(CurrentUnit.AMPS),EXPANSION_HUB.getCurrent(CurrentUnit.AMPS),Gamepad.RUMBLE_DURATION_CONTINUOUS);
            if (motorControl.isOverCurrent()) {
                gamepad1.rumble(0.5, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                gamepad2.rumble(0.5, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            }

            //autoPlacer
            //motorControl.autoPlacer.setPosition(motorControl.autoPlacer.getPosition() + gamepad2.right_stick_x / 3);

            double colorAlpha = motorControl.color.alpha();
            double pad2rumble;

            /*
            if (colorAlpha > 10000 && (motorControl.clawArm.getTargetPosition() == 0) && motorControl.clawArm.closeEnough() && !gamepad2.left_bumper && !gamepad1.square) {
                pixelInClaw = true;
            }

             */

            // rumble the gunner controller based on the claw color sensor
            if (colorAlpha > 200 && !pixelInClaw) {
                pad2rumble = Math.log10(colorAlpha) / 6;
            } else {
                pad2rumble = 0;
            }
            gamepad2.rumble(pad2rumble, pad2rumble, Gamepad.RUMBLE_DURATION_CONTINUOUS);

            // update RR, update motor controllers


            // TELEMETRY

            TelemetryPacket packet = new TelemetryPacket();
            MecanumDrive.drawRobot(packet.fieldOverlay(), drive.pose); //new Pose2d(new Vector2d(IN_PER_TICK * drive.pose.trans.x,IN_PER_TICK * drive.pose.trans.y), drive.pose.rot)

            updateAsync(packet);
            drive.updatePoseEstimate();
            motorControl.update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            double loopTimeMs = loopTime.milliseconds();

            if (motorControl.isOverCurrent() || true) {
                telemetry.addLine("!!! OVER CURRENT !!!");
                telemetry.addData("CONTROL HUB CURRENT (A)", CONTROL_HUB.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("EXPANSION HUB CURRENT (A)", EXPANSION_HUB.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("ARM CURRENT", motorControl.clawArm.motor.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("SLIDE CURRENT", motorControl.slide.motor.getCurrent(CurrentUnit.AMPS));
            }


            if (showPoseTelemetry) {
                telemetry.addLine("--- Pose ---");
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading.log());
            }
            if (showLoopTimes) {
                telemetry.addLine("--- Loop Times ---");
                telemetry.addData("loopTimeMs", loopTimeMs);
                telemetry.addData("loopTimeHz", 1000.0 / loopTimeMs);
            }
            if (showMotorTelemetry) {
                telemetry.addLine("--- Motors ---");
                telemetry.addData("armTarget", motorControl.clawArm.getTargetPosition());
                telemetry.addData("armPosition", motorControl.clawArm.motor.getCurrentPosition());
                telemetry.addData("slideTarget", motorControl.slide.getTargetPosition());
                telemetry.addData("slidePosition", motorControl.slide.motor.getCurrentPosition());
                telemetry.addData("clawPower", motorControl.claw.getPosition());
                telemetry.addData("hookPower", motorControl.hookArm.getPosition());
                telemetry.addData("colorAlpha", colorAlpha);
                telemetry.addData("colorAlphaLog", Math.log10(colorAlpha));
                telemetry.addData("touchSensor", motorControl.touch.isPressed());
                telemetry.addData("touchSensor", motorControl.magnet.isPressed());
            }
            if (showStateTelemetry) {
                telemetry.addLine("--- State Machine ---");
                telemetry.addData("stateMachineState", liftState);
                telemetry.addData("pixelInClaw", pixelInClaw);
                telemetry.addData("pixelInHook", pixelInHook);
                telemetry.addData("elapsedTime", liftTimer.milliseconds());
                telemetry.addData("driveAction", driveActionRunning());
            }
            if (showCameraTelemetry) {
                telemetry.addLine("--- Camera ---");
                if (whitePixelProcessor.getDetectedPixel() != null) {
                    telemetry.addData("pixelX", whitePixelProcessor.getDetectedPixel().x);
                    telemetry.addData("pixelY", whitePixelProcessor.getDetectedPixel().y);
                    telemetry.addData("pixelWidth", whitePixelProcessor.getDetectedPixel().width);
                }
            }
            telemetry.update();
        }
    }

        Action pixelToHook () {
            return new ParallelAction(new SequentialAction(
                    motorActions.pixelToHook(),
                    new InstantAction(() -> {
                        pixelInClaw = false;
                        pixelInHook = true;
                    })
            ),
                    new SequentialAction(
                            moveBack3In()
                    ));
        }

        Action placePixel (input input){
            return new SequentialAction(
                    motorActions.hookToBackdrop(),
                    (telemetryPacket -> padRelease()),
                    motorActions.placeSecondPixel(),
                    (telemetryPacket -> padRelease()),
                    motorActions.returnHook(),
                    new InstantAction(() -> pixelInHook = false)
            );

        }
        // TODO: probably not needed, just make a normal action
        interface input {
            boolean isPressed();
        }
        Action waitForInput (input input){
            return telemetryPacket -> input.isPressed();
        }

        boolean padRelease () {
            return !((gamepad2.right_trigger > 0.25) && previousGamepad2.right_trigger < 0.25);
        }

        Action moveBack3In() {
            return new SequentialAction(new MoveBackAction(),
                    new SleepAction(1.5),
                    new StopMovingAction());
        }

        class MoveBackAction implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!gamepad2.left_bumper) {
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(
                                            -1,
                                            0
                                    ),
                                    0
                            )
                    );
                }
                    return false;

            }
        }
        class StopMovingAction implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    0,
                                    0
                            ),
                            0
                    )
            );
                return false;
         }
        }

        boolean driveActionRunning() {
            return containsDriveAction(runningActions);
        }
        boolean containsDriveAction(List<Action> actions) {
            for (Action action : actions) {
                if (action.getClass() == MecanumDrive.FollowTrajectoryAction.class || action.getClass() == MoveBackAction.class || action.getClass() == StopMovingAction.class) {
                    return true;
                }
                if (action.getClass() == SequentialAction.class) {
                    SequentialAction sequentialAction = (SequentialAction) action;
                    return containsDriveAction(sequentialAction.getInitialActions());
                    }
                }
            return false;
        }

}

