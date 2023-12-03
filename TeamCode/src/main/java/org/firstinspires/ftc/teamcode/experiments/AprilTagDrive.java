package org.firstinspires.ftc.teamcode.experiments;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Helpers;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagDrive extends MecanumDrive {
    @Config
    public static class Params {
        // distance FROM robot center TO camera (inches)
        // TODO: tune
        public static Vector2d cameraOffset = new Vector2d(
                -9,
                4);
        public static double cameraYawOffset = Math.toRadians(180); // TODO: tune
        /*
         * Q model covariance (trust in model), default 0.1
         * R sensor covariance (trust in sensor), default 0.4
         */
        public static double kalmanFilterQ = 0.1;
        public static double kalmanFilterR = 0.4;
    }

    public static final Params PARAMS = new Params();
    final AprilTagProcessor aprilTag;
    final KalmanFilter.Vector2dKalmanFilter posFilter;
    Pose2d aprilPose;
    Pose2d localizerPose;
    Vector2d filteredVector;
    boolean shouldTagCorrect = false;
    public AprilTagDrive(HardwareMap hardwareMap, Pose2d pose, AprilTagProcessor aprilTag) {
        super(hardwareMap, pose);
        this.aprilTag = aprilTag;
        this.posFilter = new KalmanFilter.Vector2dKalmanFilter(PARAMS.kalmanFilterQ, PARAMS.kalmanFilterR);

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
        if (aprilVector != null && shouldTagCorrect) { // && aprilVector.plus(localizerPose.position.times(-1)).norm() < 24 // TODO: replace, removed for initial testing
            // TODO: apriltags unreliable at higher speeds? speed limit? global shutter cam? https://discord.com/channels/225450307654647808/225451520911605765/1164034719369941023
            filteredVector = posFilter.update(twist.value(), aprilVector);
            pose = new Pose2d(aprilVector, localizerPose.heading);
            shouldTagCorrect = false; // TODO disable
        } else {
            filteredVector = posFilter.update(twist.value(), localizerPose.position);
            pose = localizerPose;
        }

        //pose = new Pose2d(filteredVector,localizerPose.heading);

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
        Vector2d averagePos = new Vector2d(0,0); // starting pose to add the rest to
        if (currentDetections.isEmpty()) return null; // if we don't see any tags, give up (USES NEED TO HANDLE NULL)
        Vector2d RobotPos;

        // Step through the list of detections and calculate the robot position from each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && !detection.metadata.name.contains("Wall")) { // TODO: Change if we want to use wall tags?
                Vector2d tagPos = Helpers.toVector2d(detection.metadata.fieldPosition); // SDK builtin tag position
                double tagHeading = Helpers.quarternionToHeading(detection.metadata.fieldOrientation); // SDK builtin tag heading

                RobotPos = calculateRobotPosFromTag(tagPos, tagHeading,localizerPose.heading.log(), detection); // calculate the robot position from the tag position

                // we're going to get the average here by adding them all up and dividing by the number of detections
                // we do this because the backdrop has 3 tags, so we get 3 positions
                // hopefully by averaging them we can get a more accurate position
                averagePos = averagePos.plus(RobotPos);

            }
        }   // end for() loop
        // divide by the number of detections to get the true average, as explained earlier

        return averagePos.div(currentDetections.size());
    }

    @NonNull
    private static Vector2d calculateRobotPosFromTag(Vector2d tagPos, double tagHeading, double imuHeading, AprilTagDetection detection) {
        // TODO: I don't actually know trig, this is probably terrible
        double xPos;
        double yPos;
        //if (Math.abs(Math.toDegrees((imuHeading - PARAMS.cameraYawOffset) - tagHeading)) - 5 > 0) { // if the robot isn't within half a degree of straight up
            double tagRelHeading = imuHeading - PARAMS.cameraYawOffset + Math.toRadians(detection.ftcPose.bearing) - tagHeading;
            Vector2d camGlobalOffset = new Vector2d(
                    PARAMS.cameraOffset.x * Math.cos(-imuHeading) - PARAMS.cameraOffset.y * Math.sin(-imuHeading),
                    PARAMS.cameraOffset.x * Math.sin(-imuHeading) + PARAMS.cameraOffset.y * Math.cos(-imuHeading));
            xPos = tagPos.x - (Math.cos(tagRelHeading) * detection.ftcPose.y) - camGlobalOffset.x;
            yPos = tagPos.y - (Math.sin(tagRelHeading) * detection.ftcPose.y) - camGlobalOffset.y;
        /*} else {
            xPos = (tagPos.x - detection.ftcPose.y) - PARAMS.cameraOffset.x; // TODO; this will ONLY work for the backdrop tags
            yPos = (tagPos.y - detection.ftcPose.x) - PARAMS.cameraOffset.y;
        }*/

        // take the tag's field position & subtract it from the position relative to camera to get the camera's position
        // TODO: probably wrong, I don't quite comprehend this math
        //Vector2d globalCameraPos = tagPos.minus(Helpers.rotateVector(Helpers.toVector2d(Helpers.tagPoseToVectorF(detection.ftcPose)),-(localizerPose.heading.log() + PARAMS.cameraYawOffset)));


        // use the offsets to get the robot's position from the camera's position
        //Vector2d RobotPos = globalCameraPos.plus(PARAMS.cameraOffset.unaryMinus());

        // TODO: will not work for diff tags then backdrop
        //return Helpers.rotateVector(new Vector2d(xPos, yPos), tagHeading - Math.toRadians(180));
        return new Vector2d(xPos, yPos);
    }


    public void correctWithTag() {
        shouldTagCorrect = true;
    }
    public Action CorrectWithTagAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                correctWithTag();
                return false;
            }
        };
    }
}
