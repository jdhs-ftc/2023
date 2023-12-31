package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ActionOpMode;
import org.firstinspires.ftc.teamcode.experimentsSemiBroken.AprilTagDrive;
import org.firstinspires.ftc.teamcode.helpers.Helpers.RaceParallelCommand;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.motor.MotorActions;
import org.firstinspires.ftc.teamcode.motor.MotorControl;

/**
 * Used to test autonomous quickly when I don't want spike mark select, especially apriltag testing
 */
@Autonomous(name = "TestAutoOpMode", group = "Test")
@Config
public class TestAutoOpMode extends ActionOpMode {
    @Override
    public void runOpMode() {
        VisionHelper vision = new VisionHelper(hardwareMap, PoseStorage.Team.RED);
        /*
        // Initialize the tag detector
        // Lens intrinsics carefully calibrated for the random old webcam that has a ball tilt, NOT
        // for the logitech
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(517.0085f, 508.91845f, 322.364324f, 167.9933806f)
                .build();
        // CameraStreamProcessor just shows the stream on FTC Dashboard
        CameraStreamProcessor cameraStreamProcessor = new CameraStreamProcessor();

        // Initialize vision and get camera
        new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors( aprilTag, cameraStreamProcessor)
                .build();

         */
        // Initalize the basic motor control, base PID loops etc
        MotorControl motorControl = new MotorControl(hardwareMap);
        // MotorActions wraps motor control in RR actions that I can use in trajectories
        MotorActions motorActions = new MotorActions(motorControl);

        // AprilTagDrive is my experimental localizer that calibrates with apriltags but still
        // uses RR localization when it can't see them
        AprilTagDrive drive = new AprilTagDrive(hardwareMap,
                new Pose2d(-60, -36, Math.toRadians(180)), vision);

        waitForStart();

        if (isStopRequested()) return;

        vision.switchFront();

        while (opModeIsActive() && !isStopRequested()) {
            // Trajectories are technically sequentialactions, by casting them it's technically
            // possible to get data about them by getting the first or second actions in the
            // sequence
            Action traj =
                     drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .afterTime(2,vision.switchBackAction())
                            .splineTo(new Vector2d(48,-36), Math.toRadians(0))
                            .setReversed(false)
                            .setTangent(Math.toRadians(180))
                            .afterTime(2,vision.switchFrontAction())
                            .splineTo(new Vector2d(-60,-36), Math.toRadians(180))
                            .build();
            /* THIS is the important part:
             I use RaceParallelCommand to run the trajectory and update the motor actions simultaneously;
             RaceParallelCommand runs 2 actions in parallel and exits as soon as one action is done
            MotorActions in my trajectories just change variables in the motor control

            This lets me constantly have the motors run throughout the trajectory
             */
            Actions.runBlocking(new RaceParallelCommand(
                    traj,
                    motorActions.update()
            ));
        }


        /* Save the pose to use in teleop
        Or this line would do that, if I ran this auto at comp
         */
        PoseStorage.currentPose = drive.pose;
    }

}
