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
import org.firstinspires.ftc.teamcode.Helpers;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.experimentsSemiBroken.AprilTagDrive;
import org.firstinspires.ftc.teamcode.motor.MotorActions;
import org.firstinspires.ftc.teamcode.motor.MotorControl;
import org.firstinspires.ftc.teamcode.vision.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.vision.PipelineProcessor;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDeterminationPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;
// TODO: USE APRILTAGS
/** <h3>Abstract class for vision-based autonomous</h3>
 * Every vision auto is basically the same, so we use an Abstract class here
 * this allows us to separate the vision code from the actual RR trajectories
 * to make a new auto, just make a class that implements AbstractVisionOpMode
 */
public abstract class AbstractVisionOpMode extends LinearOpMode
{
    public boolean choosable() {
        return false;
    };
    /**
     * Is this a red or a blue autonomous?
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
     * You are expected to override this method
     * This is where you could have a trajectory selection etc
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
        MotorControl motorControl = new MotorControl(hardwareMap);
        MotorActions motorActions = new MotorActions(motorControl);
        Actions.runBlocking(motorActions.claw.grab());
        motorControl.slide.setTargetPosition(-100);


        pipeline = new TeamPropDeterminationPipeline(telemetry);
        pipeline.setBlue(team() == PoseStorage.Team.BLUE);
        pipelineProcessor = new PipelineProcessor(pipeline);

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(517.0085f, 508.91845f, 322.364324f, 167.9933806f)
                .build();

        CameraStreamProcessor cameraStreamProcessor = new CameraStreamProcessor();

        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(pipelineProcessor, aprilTag, cameraStreamProcessor)
                .build();

        FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor,30);





        AprilTagDrive drive = new AprilTagDrive(hardwareMap, startPose(), aprilTag);
        //MecanumDrive drive = new MecanumDrive(hardwareMap, startPose()); // TODO: THIS WILL BREAK THINGS

        Action trajLeft = trajLeft(drive, motorActions);
        Action trajCenter = trajCenter(drive, motorActions);
        Action trajRight = trajRight(drive, motorActions);


        motorControl.claw.setPosition(0.7);

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


            // Don't burn CPU cycles busy-looping in this sample
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

        motorControl.slide.motor.setMode(STOP_AND_RESET_ENCODER);
        motorControl.slide.motor.setMode(RUN_TO_POSITION);
        motorControl.activatePreset(MotorControl.combinedPreset.IDLE);


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

        PoseStorage.currentPose = drive.pose;
        PoseStorage.currentTeam = team();
    }
}