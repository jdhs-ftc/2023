package org.firstinspires.ftc.teamcode.auto.red.RightRed2plus4;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AbstractVisionOpMode;
import org.firstinspires.ftc.teamcode.experimentsSemiBroken.AprilTagDrive;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.motor.MotorActions;


@Autonomous(preselectTeleOp = "Teleop Field Centric", name = "2+4 Right Red, Far Park, Left Pixel", group = "Blue")

public class FarParkLeftPixel extends AbstractVisionOpMode {
    /**
     * Is this a red or a blue autonomous?
     *
     * @return the team
     */
    @Override
    public PoseStorage.Team team() {
        return PoseStorage.Team.RED;
    }

    /**
     * Starting Position of the trajectories
     *
     * @return the starting pose
     */
    @Override
    public Pose2d startPose() {
        return new Pose2d(12,-62,Math.toRadians(-90));
    }

    @Override
    public Action trajLeft(AprilTagDrive drive, MotorActions motorActions) {
        return drive.actionBuilder(drive.pose) // new Pose2d(-36,-62,Math.toRadians(-90))
                .setReversed(true)
                // GOTO GROUND PIXEL
                .splineToConstantHeading(new Vector2d(15,-47), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(6,-32, Math.toRadians(180)), Math.toRadians(0))
                .endTrajectory()
                .stopAndAdd(motorActions.claw.release())


                // GOTO BACKBOARD START
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .strafeTo(new Vector2d(52,-41))
                .endTrajectory()
                .stopAndAdd(motorActions.autoPlace())
                .endTrajectory()
                /*
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(55,-16),Math.toRadians(0))

                /*
                // GOTO STACK
                .setTangent(Math.toRadians(180))
                .setReversed(false)
                .afterTime(1,drive.vHelper.swit chFrontAction())
                .splineTo(new Vector2d(20, -56), Math.toRadians(180))
                .splineTo(new Vector2d(-20, -56), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-61,-35,Math.toRadians(180)), Math.toRadians(180))
                .endTrajectory()
                .stopAndAdd(motorActions.claw.grab())


                // GOTO BACKBOARD
                .setReversed(true)
                .afterTime(0.5, motorActions.pixelToHook())
                .afterTime(2, drive.vHelper.switchBackAction())
                .splineToConstantHeading(new Vector2d(-36, -10), Math.toRadians(0))
                .splineTo(new Vector2d(20, -10), Math.toRadians(0))
                //.afterTime(0.5, drive.CorrectWithTagAction())
                .splineToSplineHeading(new Pose2d(50,-33, Math.toRadians(180)), Math.toRadians(0))
                .stopAndAdd(motorActions.placeTwoPixel())
                //.endTrajectory()
                /*

                // GOTO STACK
                .setTangent(Math.toRadians(180))
                .setReversed(false)
                .splineTo(new Vector2d(20, -60), Math.toRadians(180))
                .splineTo(new Vector2d(-36, -60), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-64,-36,Math.toRadians(180)), Math.toRadians(180))
                .endTrajectory()
                .stopAndAdd(motorActions.claw.grab())


                // GOTO BACKBOARD
                .setReversed(true)
                .afterTime(0.5, motorActions.pixelToHook())
                .splineToConstantHeading(new Vector2d(-36, -14), Math.toRadians(0))
                .splineTo(new Vector2d(20, -14), Math.toRadians(0))
                //.afterTime(0.5, drive.CorrectWithTagAction())
                .splineToSplineHeading(new Pose2d(56.5,-33, Math.toRadians(180)), Math.toRadians(0))
                .stopAndAdd(new ParallelAction(motorActions.autoPlace(),motorActions.placePixel()))
                .endTrajectory()

                 */

                .waitSeconds(2)

                .build();
    }
    @Override
    public Action trajCenter(AprilTagDrive drive, MotorActions motorActions) {
       // THIS IS THE GOOD ONE
            return drive.actionBuilder(new Pose2d(12, -62, Math.toRadians(-90))) // new Pose2d(-36,-62,Math.toRadians(-90))
                    .setReversed(true)
                    // GOTO GROUND PIXEL
                    .splineToConstantHeading(new Vector2d(12, -18), Math.toRadians(90))
                    .endTrajectory()
                    .stopAndAdd(motorActions.claw.release())
                    .strafeTo(new Vector2d(12, -14))


                    // GOTO BACKBOARD START
                    .setReversed(true)

                    .setTangent(Math.toRadians(0))

                    .splineToLinearHeading(new Pose2d(59, -42.5, Math.toRadians(180)), Math.toRadians(0)) // RED SIDE 10 INCH FURTHER DOWN
                    .endTrajectory()
                    .stopAndAdd(motorActions.autoPlace())
                    .endTrajectory()
                    /*
                    .setTangent(180)
                    .splineToConstantHeading(new Vector2d(55,-16),Math.toRadians(0))

                    /*
                    // GOTO STACK
                    .setTangent(Math.toRadians(180))
                    .setReversed(false)
                    .afterTime(1, drive.vHelper.switchFrontAction())
                    .splineTo(new Vector2d(20, -60), Math.toRadians(180)) // 55 for blue 60 for red
                    .splineTo(new Vector2d(-25, -60), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-57.5, -31, Math.toRadians(180)), Math.toRadians(180))
                    .endTrajectory()
                    .stopAndAdd(motorActions.claw.grab())


                    // GOTO BACKBOARD
                    .setReversed(true)
                    .afterTime(0, motorActions.pixelToHook())
                    .afterTime(2, drive.vHelper.switchBackAction())
                    .splineToConstantHeading(new Vector2d(-36, -9), Math.toRadians(0))
                    .splineTo(new Vector2d(20, -9), Math.toRadians(0))
                    //.afterTime(0.5, drive.CorrectWithTagAction())
                    .splineToSplineHeading(new Pose2d(58, -33, Math.toRadians(180)), Math.toRadians(0))
                    .stopAndAdd(motorActions.placeTwoPixel())
                    //.endTrajectory()
                    /*

                    // GOTO STACK
                    .setTangent(Math.toRadians(180))
                    .setReversed(false)
                    .splineTo(new Vector2d(20, -60), Math.toRadians(180))
                    .splineTo(new Vector2d(-36, -60), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-64,-36,Math.toRadians(180)), Math.toRadians(180))
                    .endTrajectory()
                    .stopAndAdd(motorActions.claw.grab())


                    // GOTO BACKBOARD
                    .setReversed(true)
                    .afterTime(0.5, motorActions.pixelToHook())
                    .splineToConstantHeading(new Vector2d(-36, -14), Math.toRadians(0))
                    .splineTo(new Vector2d(20, -14), Math.toRadians(0))
                    //.afterTime(0.5, drive.CorrectWithTagAction())
                    .splineToSplineHeading(new Pose2d(56.5,-33, Math.toRadians(180)), Math.toRadians(0))
                    .stopAndAdd(new ParallelAction(motorActions.autoPlace(),motorActions.placePixel()))
                    .endTrajectory()

                     */

                    .waitSeconds(2)

                    .build();

    }

    @Override
    public Action trajRight(AprilTagDrive drive, MotorActions motorActions) {

        return drive.actionBuilder(drive.pose) // new Pose2d(-36,-62,Math.toRadians(-90))
                .setReversed(true)
                // GOTO GROUND PIXEL
                .splineToConstantHeading(new Vector2d(24,-48), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(24,-21), Math.toRadians(90))
                .endTrajectory()
                .stopAndAdd(motorActions.claw.release())
                .strafeTo(new Vector2d(23,-16))


                // GOTO BACKBOARD START
                .setReversed(true)
                .setTangent(Math.toRadians(0))

                .splineToLinearHeading(new Pose2d(50,-47,Math.toRadians(180)), Math.toRadians(0))
                .endTrajectory()
                .stopAndAdd(motorActions.autoPlace())
                .endTrajectory()
                /*
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(55,-16),Math.toRadians(0))

                /*
                // GOTO STACK
                .setTangent(Math.toRadians(180))
                .setReversed(false)
                .afterTime(1,drive.vHelper.switchFrontAction())
                .splineTo(new Vector2d(20, -56), Math.toRadians(180))
                .splineTo(new Vector2d(-20, -56), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-61,-35,Math.toRadians(180)), Math.toRadians(180))
                .endTrajectory()
                .stopAndAdd(motorActions.claw.grab())


                // GOTO BACKBOARD
                .setReversed(true)
                .afterTime(0.5, motorActions.pixelToHook())
                .afterTime(2, drive.vHelper.switchBackAction())
                .splineToConstantHeading(new Vector2d(-36, -10), Math.toRadians(0))
                .splineTo(new Vector2d(20, -10), Math.toRadians(0))
                //.afterTime(0.5, drive.CorrectWithTagAction())
                .splineToSplineHeading(new Pose2d(50,-33, Math.toRadians(180)), Math.toRadians(0))
                .stopAndAdd(motorActions.placeTwoPixel())
                //.endTrajectory()
                /*

                // GOTO STACK
                .setTangent(Math.toRadians(180))
                .setReversed(false)
                .splineTo(new Vector2d(20, -60), Math.toRadians(180))
                .splineTo(new Vector2d(-36, -60), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-64,-36,Math.toRadians(180)), Math.toRadians(180))
                .endTrajectory()
                .stopAndAdd(motorActions.claw.grab())


                // GOTO BACKBOARD
                .setReversed(true)
                .afterTime(0.5, motorActions.pixelToHook())
                .splineToConstantHeading(new Vector2d(-36, -14), Math.toRadians(0))
                .splineTo(new Vector2d(20, -14), Math.toRadians(0))
                //.afterTime(0.5, drive.CorrectWithTagAction())
                .splineToSplineHeading(new Pose2d(56.5,-33, Math.toRadians(180)), Math.toRadians(0))
                .stopAndAdd(new ParallelAction(motorActions.autoPlace(),motorActions.placePixel()))
                .endTrajectory()

                 */

                .waitSeconds(2)

                .build();
    }
}
