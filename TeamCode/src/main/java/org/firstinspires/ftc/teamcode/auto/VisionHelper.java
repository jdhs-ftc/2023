package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.helpers.vision.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.helpers.vision.PipelineProcessor;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDeterminationPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionHelper {
    public AprilTagProcessor aprilTagBack;
    public AprilTagProcessor aprilTagFront;

    public TeamPropDeterminationPipeline pipeline;
    PipelineProcessor pipelineProcessor;
    WebcamName backCam;
    WebcamName frontCam;
    VisionPortal myVisionPortal;
    public boolean frontCamActive = false;
   PtzControl.PanTiltHolder backCamPtz = new PtzControl.PanTiltHolder();
   PtzControl.PanTiltHolder frontCamPtz = new PtzControl.PanTiltHolder();
    public VisionHelper(HardwareMap hardwareMap, PoseStorage.Team team) {
        backCamPtz.tilt = 0;
        frontCamPtz.tilt = 180;

        // Initialize prop detector
        pipeline = new TeamPropDeterminationPipeline();
        // The auto that's implementing this will have set the team, so we tell the pipeline which color of prop to look for
        // Honestly not sure why I have this converted to boolean
        pipeline.setBlue(team == PoseStorage.Team.BLUE);

        // Convert that EOCV pipeline into a new processor using this helper class
        pipelineProcessor = new PipelineProcessor(pipeline);

        aprilTagBack = new AprilTagProcessor.Builder()
                .setLensIntrinsics(517.0085f, 508.91845f, 322.364324f, 167.9933806f)
                .build();

        aprilTagFront = new AprilTagProcessor.Builder()
                // TODO: CALIBRATE .setLensIntrinsics(...)
                .setLensIntrinsics(721.5303734f, 717.4905364f, 310.2800085f, 235.908762f)
                .build();
        CameraStreamProcessor cameraStreamProcessor = new CameraStreamProcessor();

        backCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        frontCam = hardwareMap.get(WebcamName.class, "Webcam 2");



        myVisionPortal = new VisionPortal.Builder()
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(640,480))
                .setCamera(ClassFactory.getInstance()
                        .getCameraManager().nameForSwitchableCamera(backCam, frontCam))
                .addProcessors(aprilTagBack, aprilTagFront, pipelineProcessor, cameraStreamProcessor)
                .enableLiveView(true)
                .build();



        FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor,30);
    }

    public boolean initLoop(Telemetry telemetry) {
        if (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Vision loading...");
            return true;
        } else {
            switchBack();
            myVisionPortal.setProcessorEnabled(pipelineProcessor, true);
            telemetry.addData("position", pipeline.getAnalysis());
            telemetry.addData("confidence", pipeline.getConfidence());
            return false;
        }

    }
    public void switchBack() {
        myVisionPortal.setActiveCamera(backCam);
        myVisionPortal.setProcessorEnabled(aprilTagBack, true);
        myVisionPortal.setProcessorEnabled(aprilTagFront, false);
        frontCamActive = false;
    }
    public void switchFront() {
        myVisionPortal.setActiveCamera(frontCam);
        myVisionPortal.setProcessorEnabled(aprilTagBack, false);
        myVisionPortal.setProcessorEnabled(aprilTagFront, true);
        frontCamActive = true;
    }
    public void setPropDetection(boolean enabled) {
        myVisionPortal.setProcessorEnabled(pipelineProcessor, enabled);
    }

    public Action switchBackAction() {
        return new InstantAction(this::switchBack);

    }
    public Action switchFrontAction() {
        return new InstantAction(this::switchFront);
    }

}
