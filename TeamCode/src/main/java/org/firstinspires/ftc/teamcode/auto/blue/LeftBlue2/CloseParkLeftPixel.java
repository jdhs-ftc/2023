package org.firstinspires.ftc.teamcode.auto.blue.LeftBlue2;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.auto.AbstractVisionOpMode;
import org.firstinspires.ftc.teamcode.experimentsSemiBroken.AprilTagDrive;
import org.firstinspires.ftc.teamcode.motor.MotorActions;

@Autonomous(preselectTeleOp = "Teleop Field Centric", name = "Left Blue, Close Park, Left Pixel", group = "Blue")
@Disabled
public class CloseParkLeftPixel extends AbstractVisionOpMode {
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
        return new Pose2d(12,60,Math.toRadians(90));
    }

    @Override
    public Action trajLeft(AprilTagDrive drive, MotorActions motorActions) {
        return drive.actionBuilder(drive.pose)
                .setReversed(true)
                // GOTO GROUND PIXEL
                .splineToSplineHeading(new Pose2d(29,32, Math.toRadians(180)), Math.toRadians(0))
                .endTrajectory()
                .stopAndAdd(drive.CorrectWithTagAction())
                .stopAndAdd(() -> motorActions.motorControl.claw.setPosition(0.95))
                // GOTO BACKBOARD
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(53,40, Math.toRadians(180)), Math.toRadians(0))

                .stopAndAdd(new SequentialAction(
                        new InstantAction(() -> {motorActions.motorControl.autoPlacer.setPosition(0.5);}),
                        new SleepAction(0.5),
                        new InstantAction(() -> {motorActions.motorControl.autoPlacer.setPosition(1);})))

                .endTrajectory()


                // PARK
                .splineToLinearHeading(new Pose2d(60,60, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    @Override
    public Action trajCenter(AprilTagDrive drive, MotorActions motorActions) {
        return drive.actionBuilder(drive.pose)
                .setReversed(true)
                // GOTO GROUND PIXEL
                .splineToSplineHeading(new Pose2d(12,33, Math.toRadians(-90)), Math.toRadians(-90))
                .endTrajectory()
                .stopAndAdd(drive.CorrectWithTagAction())

                .stopAndAdd(telemetryPacket -> {
                    motorActions.motorControl.claw.setPosition(0.95);
                    return false;
                })


                // GOTO BACKBOARD
                .strafeTo(new Vector2d(13,35))
                .splineToSplineHeading(new Pose2d(53,31, Math.toRadians(180)), Math.toRadians(0))

                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {motorActions.motorControl.autoPlacer.setPosition(0.5); return false;},
                        new SleepAction(0.5),
                        telemetryPacket -> {motorActions.motorControl.autoPlacer.setPosition(1); return false;}))


                .endTrajectory()

                // PARK
                .splineToLinearHeading(new Pose2d(60,60, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    @Override
    public Action trajRight(AprilTagDrive drive, MotorActions motorActions) {

        return drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(8,32, Math.toRadians(180)), Math.toRadians(-90))
                .endTrajectory()
                .stopAndAdd(drive.CorrectWithTagAction())
                .stopAndAdd(telemetryPacket -> {
                    motorActions.motorControl.claw.setPosition(0.95);
                    return false;
                })
                // PLACE HERE
                .strafeTo(new Vector2d(9,32))
                .splineTo(new Vector2d(55.75,29), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {motorActions.motorControl.autoPlacer.setPosition(0.5); return false;},
                        new SleepAction(0.5),
                        telemetryPacket -> {motorActions.motorControl.autoPlacer.setPosition(1); return false;}))
                .splineToLinearHeading(new Pose2d(60,60, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }
}
