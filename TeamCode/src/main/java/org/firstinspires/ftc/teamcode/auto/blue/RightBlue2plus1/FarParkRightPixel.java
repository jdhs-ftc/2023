package org.firstinspires.ftc.teamcode.auto.blue.RightBlue2plus1;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.auto.AbstractVisionOpMode;
import org.firstinspires.ftc.teamcode.experimentsSemiBroken.AprilTagDrive;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.motor.MotorActions;

@Autonomous(preselectTeleOp = "Teleop Field Centric", name = "2+1 Right Blue, Far Park, Right Pixel", group = "Blue")
@Disabled
//TODO: DOESN'T
public class FarParkRightPixel extends AbstractVisionOpMode {
    /**
     * Is this a red or a blue autonomous?
     *
     * @return the team
     */
    @Override
    public PoseStorage.Team team() {
        return PoseStorage.Team.BLUE;
    }

    /**
     * Starting Position of the trajectories
     *
     * @return the starting pose
     */
    @Override
    public Pose2d startPose() {
        return new Pose2d(-36,62,Math.toRadians(90));
    }

    @Override
    public Action trajLeft(AprilTagDrive drive, MotorActions motorActions) {
        return drive.actionBuilder(drive.pose)
                .setReversed(true)
                // GOTO GROUND PIXEL
                .splineToConstantHeading(new Vector2d(-42,47), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-33,32, Math.toRadians(0)), Math.toRadians(-90))
                .endTrajectory()

                .stopAndAdd(motorActions.claw.release())


                // GOTO BACKBOARD
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-36, 12), Math.toRadians(0))
                .splineTo(new Vector2d(20, 12), Math.toRadians(0))
                .afterTime(0.5, drive.CorrectWithTagAction())
                .splineToSplineHeading(new Pose2d(52.5,38, Math.toRadians(180)), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(motorActions.autoPlace()))
                .endTrajectory()


                // PARK
                .splineToLinearHeading(new Pose2d(54,14, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    @Override
    public Action trajCenter(AprilTagDrive drive, MotorActions motorActions) {

        return drive.actionBuilder(drive.pose)
                .setReversed(true)
                // GOTO GROUND PIXEL
                .splineToConstantHeading(new Vector2d(-36,18), Math.toRadians(-90))
                .endTrajectory()
                .stopAndAdd(motorActions.claw.release())
                // GOTO BACKBOARD
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-28, 12), Math.toRadians(0))
                .splineTo(new Vector2d(20, 12), Math.toRadians(0))
                .afterTime(0.5, drive.CorrectWithTagAction())
                .splineToSplineHeading(new Pose2d(56.5,32, Math.toRadians(180)), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(motorActions.autoPlace()))
                .endTrajectory()


                // PARK
                .splineToLinearHeading(new Pose2d(54,14, Math.toRadians(180)), Math.toRadians(0))
                .build();

    }

    @Override
    public Action trajRight(AprilTagDrive drive, MotorActions motorActions) {

        return drive.actionBuilder(drive.pose)
                .setReversed(true)
                // GOTO GROUND PIXEL
                .splineToConstantHeading(new Vector2d(-48,21), Math.toRadians(-90))
                .endTrajectory()

                .stopAndAdd(motorActions.claw.release())


                // GOTO BACKBOARD
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-36, 12), Math.toRadians(0))
                .splineTo(new Vector2d(20, 12), Math.toRadians(0))
                .afterTime(0.5, drive.CorrectWithTagAction())
                .splineToSplineHeading(new Pose2d(56.5,26, Math.toRadians(180)), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(motorActions.autoPlace()))
                .endTrajectory()


                // PARK
                .splineToLinearHeading(new Pose2d(54,14, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }
}
