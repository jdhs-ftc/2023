package org.firstinspires.ftc.teamcode.auto.blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.auto.AbstractVisionOpMode;
import org.firstinspires.ftc.teamcode.experiments.AprilTagDrive;
import org.firstinspires.ftc.teamcode.motor.MotorControlActions;

@Autonomous
public class LeftBlueBothClosePark extends AbstractVisionOpMode {
    /**
     * Is this a red or a blue autonomous?
     *
     * @return the team
     */
    @Override
    public PoseStorage.Team team() {
        return PoseStorage.Team.BlUE;
    }

    /**
     * Starting Position of the trajectories
     *
     * @return the starting pose
     */
    @Override
    public Pose2d startPose() {
        return new Pose2d(12,60,Math.toRadians(90));
    }

    @Override
    public Action trajLeft(AprilTagDrive drive, MotorControlActions motorControlActions) {
        return drive.actionBuilder(drive.pose)
                .setReversed(true)
                // GOTO GROUND PIXEL
                .splineToSplineHeading(new Pose2d(29,32, Math.toRadians(180)), Math.toRadians(0))
                .endTrajectory()
                .stopAndAdd(drive.CorrectWithTagAction())
                .stopAndAdd(telemetryPacket -> {
                    motorControlActions.motorControl.claw.setPosition(0.95);
                    return false;
                })
                // GOTO BACKBOARD
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(50,37, Math.toRadians(180)), Math.toRadians(0))

                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {motorControlActions.motorControl.autoPlacer.setPosition(0.5); return false;},
                        new SleepAction(0.5),
                        telemetryPacket -> {motorControlActions.motorControl.autoPlacer.setPosition(1); return false;}))

                .endTrajectory()


                // PARK
                .splineToLinearHeading(new Pose2d(60,60, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    @Override
    public Action trajCenter(AprilTagDrive drive, MotorControlActions motorControlActions) {
        return drive.actionBuilder(drive.pose)
                .setReversed(true)
                // GOTO GROUND PIXEL
                .splineToSplineHeading(new Pose2d(12,33, Math.toRadians(-90)), Math.toRadians(-90))
                .endTrajectory()
                .stopAndAdd(drive.CorrectWithTagAction())

                .stopAndAdd(telemetryPacket -> {
                    motorControlActions.motorControl.claw.setPosition(0.95);
                    return false;
                })


                // GOTO BACKBOARD
                .strafeTo(new Vector2d(13,35))
                .splineToSplineHeading(new Pose2d(50.25,31, Math.toRadians(180)), Math.toRadians(0))

                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {motorControlActions.motorControl.autoPlacer.setPosition(0.5); return false;},
                        new SleepAction(0.5),
                        telemetryPacket -> {motorControlActions.motorControl.autoPlacer.setPosition(1); return false;}))


                .endTrajectory()

                // PARK
                .splineToLinearHeading(new Pose2d(60,60, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    @Override
    public Action trajRight(AprilTagDrive drive, MotorControlActions motorControlActions) {

        return drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(8,32, Math.toRadians(180)), Math.toRadians(-90))
                .endTrajectory()
                .stopAndAdd(drive.CorrectWithTagAction())
                .stopAndAdd(telemetryPacket -> {
                    motorControlActions.motorControl.claw.setPosition(0.95);
                    return false;
                })
                // PLACE HERE
                .strafeTo(new Vector2d(9,32))
                .splineTo(new Vector2d(50,25), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {motorControlActions.motorControl.autoPlacer.setPosition(0.5); return false;},
                        new SleepAction(0.5),
                        telemetryPacket -> {motorControlActions.motorControl.autoPlacer.setPosition(1); return false;}))
                .splineToLinearHeading(new Pose2d(60,60, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }
}
