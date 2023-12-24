/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.experimentsSemiBroken.AprilTagDrive;
import org.firstinspires.ftc.teamcode.helpers.Helpers;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.helpers.vision.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.helpers.vision.PipelineProcessor;
import org.firstinspires.ftc.teamcode.motor.MotorActions;
import org.firstinspires.ftc.teamcode.motor.MotorControl;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDeterminationPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;
// TODO: USE APRILTAGS
/** <h3>Abstract class for vision-based autonomous</h3>
 * Every vision auto is basically the same, so we use an Abstract class here
 * this allows us to separate the vision code from the actual RR trajectories
 * <p>
 * To make a new auto, just make a class that implements AbstractVisionOpMode
 */
public abstract class AbstractVisionOpMode extends LinearOpMode
{
    public boolean choosable() {
        return false;
    };
    /**
     * Declare what team this autonomous is for
     * This lets me use that team to automatically select the right vision
     * Additionally it lets me know which way the drivers think is forward for field centric drive later
     * @return the team
     */
    public abstract PoseStorage.Team team();
    /**
     * Starting Position of the trajectories
     * @return the starting pose
     */
    public abstract Pose2d startPose();
    public abstract Action trajLeft(AprilTagDrive drive, MotorActions motorActions);
    public abstract Action trajCenter(AprilTagDrive drive, MotorActions motorActions);
    public abstract Action trajRight(AprilTagDrive drive, MotorActions motorActions);

    /**
     * The INIT-loop
     * <p>
     * This is where trajectory selection could happen like with ChooseableAuto.java, however I never implemented it
     * @return true to recalculate trajectories
     */
    public boolean initLoop() {
        return false;
    };
    OpenCvWebcam webcam;
    TeamPropDeterminationPipeline pipeline;
    TeamPropDeterminationPipeline.PropPosition snapshotAnalysis = TeamPropDeterminationPipeline.PropPosition.LEFT; // default
    AprilTagProcessor aprilTag;
    PipelineProcessor pipelineProcessor;
    VisionPortal myVisionPortal;
    public boolean parkClose = true;
    public boolean pixelPlaceLeft = true;
    public AtomicBoolean ready = new AtomicBoolean(!choosable());
    public AtomicBoolean reloadTrajectories = new AtomicBoolean(false);

    @Override
    public void runOpMode()
    {
        // Initalize the basic motor control, base PID loops etc
        MotorControl motorControl = new MotorControl(hardwareMap);
        // MotorActions wraps motor control in RR actions that I can use in trajectories
        MotorActions motorActions = new MotorActions(motorControl);
        // Grab the preloaded pixel with the claw
        Actions.runBlocking(motorActions.claw.grab());
        // Helps fix slide drifting up issues, sets a really low target pos then reset
        // Not sure why slide drifts up, could be encoder issue or RTP jank
        motorControl.slide.setTargetPosition(-100);


        // Initialize prop detector
        pipeline = new TeamPropDeterminationPipeline(telemetry);
        // The auto that's implementing this will have set the team, so we tell the pipeline which color of prop to look for
        // Honestly not sure why I have this converted to boolean
        pipeline.setBlue(team() == PoseStorage.Team.BLUE);

        // Convert that EOCV pipeline into a new processor using this helper class
        pipelineProcessor = new PipelineProcessor(pipeline);


        // Create new apriltag processor using our tuned constants (TUNING FOR OLD WACK CAMERA NOT LOGITECH)
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(517.0085f, 508.91845f, 322.364324f, 167.9933806f)
                .build();

        // CameraStreamProcessor is a helper class that lets us see the camera stream on FTC Dashboard
        CameraStreamProcessor cameraStreamProcessor = new CameraStreamProcessor();

        // Initialize with all 3 processors
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(pipelineProcessor, aprilTag, cameraStreamProcessor)
                .build();
        // Here we use said processor to display the preview on FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor,30);

        // Init our custom version of MecanumDrive that corrects with apriltags
        AprilTagDrive drive = new AprilTagDrive(hardwareMap, startPose(), aprilTag);
        //MecanumDrive drive = new MecanumDrive(hardwareMap, startPose());

        // Initalize the trajectories based on the implementations of the abstract classes
        // Also we init here assuming it'll take a bit, though it seems fast in 1.0
        Action trajLeft = trajLeft(drive, motorActions);
        Action trajCenter = trajCenter(drive, motorActions);
        Action trajRight = trajRight(drive, motorActions);


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            /*
            telemetry.addData("Realtime analysis", pipeline.getAnalysis()); // Commented out: pipeline has it's own telemetry
            telemetry.update();
             */

            // This stuff is for a telemetry selection thing I never ended up using
            // It doesn't matter
            if (!ready.get()) {
                telemetry.addAction(() -> {
                    parkClose = !parkClose;
                    reloadTrajectories.set(true);
                });
                if (parkClose) {
                    telemetry.addData("Parking:", "Close");
                } else {
                    telemetry.addData("Parking", "Far");
                }
                telemetry.addAction(() -> {
                    pixelPlaceLeft = !pixelPlaceLeft;
                    reloadTrajectories.set(true);
                });
                if (pixelPlaceLeft) {
                    telemetry.addData("Pixel Place:", "Left");
                } else {
                    telemetry.addData("Pixel Place:", "Right");
                }
                telemetry.addAction(() -> {
                    ready.set(true);
                });
                telemetry.update();
            }
            if (reloadTrajectories.getAndSet(false)) {
                trajLeft = trajLeft(drive, motorActions);
                trajCenter = trajCenter(drive, motorActions);
                trajRight = trajRight(drive, motorActions);
            }


            // Don't burn CPU cycles busy-looping
            sleep(10);
        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        // Reset slide to fix issue as mentioned earlier
        motorControl.slide.motor.setMode(STOP_AND_RESET_ENCODER);
        motorControl.slide.motor.setMode(RUN_TO_POSITION);
        motorControl.activatePreset(MotorControl.combinedPreset.IDLE);

        // Here we use the selection from the vision to select the right trajectory
        // Then we use RaceParallelCommand to run both the trajectory and our motor updating until the trajectory ends
        switch (snapshotAnalysis)
        {
            case LEFT:
            {
                Actions.runBlocking(new Helpers.RaceParallelCommand(
                        trajLeft,
                        motorActions.update()
                ));
                break;
            }

            case CENTER:
            {
                Actions.runBlocking(new Helpers.RaceParallelCommand(
                        trajCenter,
                        motorActions.update()
                ));
                break;
            }

            case RIGHT:
            {

                Actions.runBlocking(new Helpers.RaceParallelCommand(
                        trajRight,
                        motorActions.update()
                ));
                break;
            }
        }
        // Here we save the pose and team to static variables so we can use them in teleop
        // Having the correct field centric based on our end point in auto is super useful
        PoseStorage.currentPose = drive.pose;
        PoseStorage.currentTeam = team();
    }
}