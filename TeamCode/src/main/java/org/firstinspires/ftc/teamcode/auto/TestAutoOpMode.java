package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ActionOpMode;
import org.firstinspires.ftc.teamcode.Helpers.RaceParallelCommand;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.experiments.AprilTagDrive;
import org.firstinspires.ftc.teamcode.motor.MotorControl;
import org.firstinspires.ftc.teamcode.motor.MotorControlActions;
import org.firstinspires.ftc.teamcode.vision.CameraStreamProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "TestAutoOpMode", group = "Test")
@Config
public class TestAutoOpMode extends ActionOpMode {
    @Override
    public void runOpMode() {

        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(517.0085f, 508.91845f, 322.364324f, 167.9933806f)
                .build();

        CameraStreamProcessor cameraStreamProcessor = new CameraStreamProcessor();

        new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors( aprilTag, cameraStreamProcessor)
                .build();

        MotorControl motorControl = new MotorControl(hardwareMap);
        MotorControlActions motorControlActions = new MotorControlActions(motorControl);


        AprilTagDrive drive = new AprilTagDrive(hardwareMap, new Pose2d(12, -60, Math.toRadians(-90)), aprilTag);
        SequentialAction traj =
                (SequentialAction) drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .strafeTo(new Vector2d(12,-55))
                        .splineToSplineHeading(new Pose2d(48,-36,Math.toRadians(180)), Math.toRadians(0))
                        .stopAndAdd(drive.CorrectWithTagAction())
                        .splineToConstantHeading(new Vector2d(-60,-37), Math.toRadians(180))
                .build();



        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new RaceParallelCommand(
                traj,
                motorControlActions.update()
        ));



        PoseStorage.currentPose = drive.pose;
    }

}
