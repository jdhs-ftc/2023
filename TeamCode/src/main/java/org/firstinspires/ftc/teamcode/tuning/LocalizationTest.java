package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.auto.VisionHelper;
import org.firstinspires.ftc.teamcode.experimentsSemiBroken.AprilTagDrive;
import org.firstinspires.ftc.teamcode.helpers.Helpers;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class LocalizationTest extends LinearOpMode {
    private AprilTagDetection tag;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            VisionHelper vision = new VisionHelper(hardwareMap, PoseStorage.Team.BLUE);

            AprilTagDrive drive = new AprilTagDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(180)), vision);

            while (opModeIsActive() && opModeInInit() && !isStopRequested() && vision.initLoop(telemetry)) {
                sleep(50);
            }

            waitForStart();

            vision.setPropDetection(false);

            while (opModeIsActive()) {

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                if (gamepad1.right_bumper) {
                    vision.switchBack();
                } else if (gamepad1.left_bumper) {
                    vision.switchFront();
                }

                drive.correctWithTag();
                drive.updatePoseEstimate();
                TelemetryPacket packet = new TelemetryPacket();
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                if (!drive.totalDetections.isEmpty()) {
                    tag = drive.totalDetections.get(0);
                    packet.fieldOverlay().fillCircle(tag.metadata.fieldPosition.get(0), drive.totalDetections.get(0).metadata.fieldPosition.get(1), 2);
                    packet.fieldOverlay().setAlpha(0.5);
                    packet.fieldOverlay().strokeLine(tag.metadata.fieldPosition.get(0), tag.metadata.fieldPosition.get(1),tag.metadata.fieldPosition.get(0),drive.pose.position.y);
                    packet.fieldOverlay().strokeLine(tag.metadata.fieldPosition.get(0), drive.pose.position.y,drive.pose.position.x,drive.pose.position.y);
                    packet.fieldOverlay().strokeLine(tag.metadata.fieldPosition.get(0), tag.metadata.fieldPosition.get(1),drive.pose.position.x,drive.pose.position.y);
                    packet.fieldOverlay().setAlpha(1);
                    telemetry.addData("tagFieldPosX", tag.metadata.fieldPosition.get(0));
                    telemetry.addData("tagFieldPosY", tag.metadata.fieldPosition.get(1));
                    telemetry.addData("tagHeading", Math.toDegrees(Helpers.quarternionToHeading(tag.metadata.fieldOrientation)));
                    telemetry.addData("tagRelPosX", tag.ftcPose.x);
                    telemetry.addData("tagRelPosY", tag.ftcPose.y);
                    telemetry.addData("tagRelBearing", tag.ftcPose.bearing);
                }


                telemetry.update();


                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
