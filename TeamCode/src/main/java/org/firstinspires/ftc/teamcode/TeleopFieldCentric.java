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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.motor.MotorControl;

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

    @Override
    public void runOpMode() {

        //  Initialization Period

        // Enable Performance Optimization
        //PhotonCore.enable();
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

        waitForStart();

        if (isStopRequested()) return;


        // Run Period

        while (opModeIsActive() && !isStopRequested()) {
            double prevTime = System.nanoTime(); // set to current time

            // Update the speed
            if (gamepad1.left_bumper) {
                speed = .35;
            } else if (gamepad1.right_bumper) {
                speed = 1;
            } else {
                speed = .8;
            }

            if (gamepad1.dpad_left) {
                drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(-90.0));
            }
            if (gamepad1.dpad_up) {
                fieldCentric = true;
            } else if (gamepad1.dpad_down) {
                fieldCentric = false;

            }

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * speed,
                    -gamepad1.left_stick_x * speed
            );
            //Pose2d poseEstimate = drive.pose;
            //double rotationAmount = -drive.pose.heading.log() + Math.toRadians(90.0); // Rotation2d.log() makes it into a double in radians.
            if (fieldCentric) {
                //input = new Vector2d(input.x * Math.cos(rotationAmount) - input.y * Math.sin(rotationAmount), input.x * Math.sin(rotationAmount) + input.y * Math.cos(rotationAmount));
                input = drive.pose.heading.inverse().plus(90).times(new Vector2d(-input.x, input.y)); // magic courtesy of https://github.com/acmerobotics/road-runner/issues/90#issuecomment-1722674965
            }
            Vector2d controllerHeading = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);

            if (controllerHeading.minus(new Vector2d(0.0,0.0)).norm() < 0.7) {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        input.x,
                                        input.y
                                ),
                                (gamepad1.left_trigger - gamepad1.right_trigger)
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
                //headingController.targetPosition = drive.getExternalHeading() + poleDetectionPipeline.getMaxRect().x

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



            motorControl.claw.setPower(0.5 + gamepad2.left_trigger - gamepad2.right_trigger);





            // Slide
            motorControl.slide.setTargetPosition(motorControl.slide.getTargetPosition() + (-gamepad2.left_stick_y * 20));
            if (gamepad2.y || gamepad1.y) {
                motorControl.setCurrentMode(MotorControl.combinedMode.PLACE);
            }
            if (gamepad2.b || gamepad1.b) {
                motorControl.setCurrentMode(MotorControl.combinedMode.IDLE);
            }
            if (gamepad2.a || gamepad1.a) {
                motorControl.setCurrentMode(MotorControl.combinedMode.GRAB);
            }
            if (gamepad2.x || gamepad1.x) {
                motorControl.arm.moveOut();
                motorControl.slide.setTargetPosition(400);
            }
            if (gamepad2.dpad_right && gamepad2.dpad_left) {
                motorControl.reset();
            }





            // TODO: remove
            gamepad1.rumble(CONTROL_HUB.getCurrent(CurrentUnit.AMPS),EXPANSION_HUB.getCurrent(CurrentUnit.AMPS),Gamepad.RUMBLE_DURATION_CONTINUOUS);
            drive.updatePoseEstimate();
            motorControl.update();
            // Timing
            // measure difference between current time and previous time
            double timeDifference = (System.nanoTime() - prevTime) / 1000000.0;

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
            telemetry.update();
        }
    }
}