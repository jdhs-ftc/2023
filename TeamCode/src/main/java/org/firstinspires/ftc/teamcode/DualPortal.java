package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class DualPortal extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.setMsTransmissionInterval(50);

        int[] viewIDs = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        VisionPortal portal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setLiveViewContainerId(viewIDs[0])
                .build();

        while (!isStopRequested() && portal1.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            telemetry.addLine("Waiting for portal 1 to come online");
            telemetry.update();
        }

        if (isStopRequested())
        {
            return;
        }

        VisionPortal portal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setLiveViewContainerId(viewIDs[1])
                .build();

        while (!isStopRequested() && portal2.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            telemetry.addLine("Waiting for portal 2 to come online");
            telemetry.update();
        }

        if (isStopRequested())
        {
            return;
        }

        while (!isStopRequested())
        {
            telemetry.addLine("All cameras online");
            telemetry.update();
            sleep(500);
        }
    }
}