package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagDrive extends MecanumDrive {
    public static class Params {
        // distance FROM camera TO robot center: opposite of what you think
        // TODO: tune
        public double cameraXOffset = 0;
        public double cameraYOffset = 0;
        public double cameraZOffset = 0;
    }

    public static Params PARAMS = new Params();
    AprilTagProcessor aprilTag;
    Pose2d aprilPose;
    Pose2d localizerPose;
    public AprilTagDrive(HardwareMap hardwareMap, Pose2d pose, AprilTagProcessor aprilTag) {
        super(hardwareMap, pose);
        this.aprilTag = aprilTag;
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        // RR standard: get the movement between loops from the localizer
        // RR assumes there's no way to get absolute position and gets relative between loops
        Twist2dDual<Time> twist = localizer.update();
        localizerPose = pose.plus(twist.value());
        // Get the absolute position from the camera
        Vector2d aprilVector = getVectorBasedOnTags();
        // it's possible we can't see any tags, so we need to check for null
        if (aprilVector != null) {
            // if we can see tags, we use the apriltag position
            // however apriltags don't have accurate headings so we use the localizer heading
            // localizer heading, for us and in TwoDeadWheelLocalizer, is IMU and absolute-ish
            aprilPose = new Pose2d(aprilVector, localizerPose.heading);
        }

        // basically: if we see a tag and if the localizers don't disagree TOO much
        if (aprilVector != null && aprilVector.plus(localizerPose.position.times(-1)).norm() < 24) {
            pose = aprilPose;
        } else {
            pose = localizerPose;
        }

        // rr standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        return twist.velocity().value(); // trust the existing localizer for speeds; because I don't know how to do it with apriltags
    }

    public Vector2d getVectorBasedOnTags() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        VectorF averagePose = new VectorF(0,0,0); // starting pose to add the rest to
        if (currentDetections.isEmpty()) return null; // if we don't see any tags, give up (USES NEED TO HANDLE NULL)

        // Step through the list of detections and calculate the robot position from each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && !detection.metadata.name.contains("Wall")) { // TODO: Change if we want to use wall tags?
                VectorF tagPos = detection.metadata.fieldPosition; // SDK builtin tag position

                // take the tag's field position & subtract it from the position relative to camera to get the camera's position
                // TODO: probably wrong
                VectorF estGlobalCameraPos = tagPos.subtracted(tagPoseToVectorF(detection.ftcPose));

                // use the offsets to get the robot's position from the camera's position
                VectorF RobotPos = estGlobalCameraPos.added(new VectorF(new float[] {
                        (float) PARAMS.cameraXOffset,
                        (float) PARAMS.cameraYOffset,
                        (float) PARAMS.cameraZOffset
                }));

                // we're going to get the average here by adding them all up and dividing by the number of detections
                // we do this because the backdrop has 3 tags, so we get 3 positions
                // hopefully by averaging them we can get a more accurate position
                averagePose.add(RobotPos);
            }
        }   // end for() loop
        // divide by the number of detections to get the true average, as explained earlier
        averagePose.multiplied(1.0f / currentDetections.size());

        // AprilTags use a "weird" 3d coordinate format, this converts it into the roadrunner native Vector2d
        Vector2d finalPose = Helpers.toVector2d(averagePose);
        return finalPose;
    }

    /**
     * Converts from the SDK AprilTagPoseFtc coordinates to the SDK VectorF coordinates
     * To be honest I don't understand the difference and this will almost certainly cause a bug
     * But the tag positions are in VectorF
     * @param aprilTagPoseFtc the AprilTagPoseFtc to convert
     * @return equivalent VectorF
     */
    public VectorF tagPoseToVectorF(AprilTagPoseFtc aprilTagPoseFtc) { // TODO: this is probably wrong somehow
        return new VectorF(new float[] {
                (float) aprilTagPoseFtc.x,
                (float) aprilTagPoseFtc.y,
                (float) aprilTagPoseFtc.z
        });
    }
}
