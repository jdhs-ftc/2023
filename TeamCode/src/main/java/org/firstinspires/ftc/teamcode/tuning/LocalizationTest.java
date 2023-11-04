package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Helpers;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.experiments.AprilTagDrive;
import org.firstinspires.ftc.teamcode.vision.CameraStreamProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                    .build();
            CameraStreamProcessor cameraStreamProcessor = new CameraStreamProcessor();

            VisionPortal myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(aprilTag, cameraStreamProcessor)
                    .build();

            FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor,30);

            AprilTagDrive drive = new AprilTagDrive(hardwareMap, new Pose2d(0,0,0), aprilTag);

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();


                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
                if (!aprilTag.getDetections().isEmpty()) {
                    telemetry.addData("tagFieldPosX", aprilTag.getDetections().get(0).metadata.fieldPosition.get(0));
                    telemetry.addData("tagFieldPosY", aprilTag.getDetections().get(0).metadata.fieldPosition.get(1));
                    telemetry.addData("tagHeading", Math.toDegrees(Helpers.quarternionToHeading(aprilTag.getDetections().get(0).metadata.fieldOrientation)));
                    telemetry.addData("tagRelPosX", aprilTag.getDetections().get(0).ftcPose.x);
                    telemetry.addData("tagRelPosY", aprilTag.getDetections().get(0).ftcPose.y);
                }
                telemetry.update();
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
                telemetry.addData("heading", drive.pose.heading);
                telemetry.update();
            }
        } else {
            throw new AssertionError();
        }
    }
}
