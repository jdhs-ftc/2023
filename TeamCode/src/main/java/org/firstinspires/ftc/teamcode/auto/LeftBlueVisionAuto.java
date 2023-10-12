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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.motor.MotorControl;
import org.firstinspires.ftc.teamcode.motor.MotorControlActions;
import org.firstinspires.ftc.teamcode.vision.pipelines.BlueTeamPropDeterminationPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
@TeleOp
public class LeftBlueVisionAuto extends ActionOpMode
{
    OpenCvWebcam webcam;
    BlueTeamPropDeterminationPipeline pipeline;
    BlueTeamPropDeterminationPipeline.PropPosition snapshotAnalysis = BlueTeamPropDeterminationPipeline.PropPosition.LEFT; // default

    @Override
    public void runOpMode()
    {
        MotorControl motorControl = new MotorControl(hardwareMap);
        MotorControlActions motorControlActions = new MotorControlActions(motorControl);
        runBlocking(motorControlActions.lowerClaw.grab());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new BlueTeamPropDeterminationPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 30);




        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, 63, Math.toRadians(-90)));
        drive.pose = new Pose2d(12,63, Math.toRadians(-90));
        Action trajLeft =
                drive.actionBuilder(drive.pose)
                        .strafeToSplineHeading(new Vector2d(12,35), Rotation2d.exp(Math.toRadians(0)))
                        .strafeToConstantHeading(new Vector2d(15,35))

                        .stopAndAdd(new SequentialAction(
                                motorControlActions.setCurrentMode(MotorControl.combinedMode.GRAB),
                                new SleepAction(0.25),
                                motorControlActions.lowerClaw.release(),
                                new SleepAction(0.1),
                                motorControlActions.setCurrentMode(MotorControl.combinedMode.IDLE)
                        ))

                        .strafeTo(new Vector2d(12, 35))
                        .strafeTo(new Vector2d(12, 36))
                        .splineToSplineHeading(new Pose2d(60,60,Math.toRadians(180.0000001)), Math.toRadians(0))
                        .build();

        Action trajCenter =
                drive.actionBuilder(drive.pose)
                        .strafeToSplineHeading(new Vector2d(12,35), Rotation2d.exp(Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(12,32))
                        .stopAndAdd(new SequentialAction(
                                motorControlActions.setCurrentMode(MotorControl.combinedMode.GRAB),
                                new SleepAction(0.25),
                                motorControlActions.lowerClaw.release()
                        ))
                        .strafeTo(new Vector2d(12, 36))
                        .splineToSplineHeading(new Pose2d(60,60,Math.toRadians(180.0000001)), Math.toRadians(0))
                        .build();


        Action trajRight =
                drive.actionBuilder(drive.pose)
                        .strafeToSplineHeading(new Vector2d(12,33), Rotation2d.exp(Math.toRadians(180)))
                        .strafeToConstantHeading(new Vector2d(10,33))
                        .stopAndAdd(new SequentialAction(
                                motorControlActions.setCurrentMode(MotorControl.combinedMode.GRAB),
                                new SleepAction(0.1),
                                motorControlActions.lowerClaw.release(),
                                new SleepAction(0.1),
                                motorControlActions.setCurrentMode(MotorControl.combinedMode.IDLE)
                        ))
                        .strafeTo(new Vector2d(12, 33))
                        .splineToSplineHeading(new Pose2d(60,60,Math.toRadians(180.0000001)), Math.toRadians(0))
                        .build();


        motorControl.setCurrentMode(MotorControl.combinedMode.GRAB);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            if (gamepad2.left_bumper) {
                motorControl.lowerClaw.setPower(0.8);
            }
            if (gamepad2.right_bumper) {
                motorControl.upperClaw.setPower(0.8);
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

        motorControl.setCurrentMode(MotorControl.combinedMode.IDLE);


        switch (snapshotAnalysis)
        {
            case LEFT:
            {
                runBlocking(new MotorControlActions.RaceParallelCommand(
                        trajLeft,
                        motorControlActions.update()
                ));
                break;
            }

            case RIGHT:
            {
                /*
                runBlocking(new MotorControlActions.RaceParallelCommand(
                        trajRight,
                        motorControlActions.update()
                ));

                 */
                break;
            }

            case CENTER:
            {
                /*
                runBlocking(new MotorControlActions.RaceParallelCommand(
                        trajCenter,
                        motorControlActions.update()
                ));

                 */
                break;
            }
        }

        PoseStorage.currentPose = drive.pose;
    }
}