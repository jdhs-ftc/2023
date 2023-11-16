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
public class LeftBlue2plus2ClosePark extends AbstractVisionOpMode {
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
                .splineToSplineHeading(new Pose2d(28,31, Math.toRadians(180)), Math.toRadians(0))
                .endTrajectory()

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

                // STACK
                .splineTo(new Vector2d(-12, 55), Math.toRadians(180))
                .splineTo(new Vector2d(-63,34), Math.toRadians(180))
                // GRAB PIXEL
                .stopAndAdd(telemetryPacket -> {motorControlActions.motorControl.claw.setPosition(0.7); return false;})
                .endTrajectory()
                .strafeTo(new Vector2d(-56,34))

                // CYCLE
                .stopAndAdd(motorControlActions.pixelToHookCycle())
                .endTrajectory()
                .strafeTo(new Vector2d(-63,34))
                // GRAB PIXEL
                .stopAndAdd(telemetryPacket -> {motorControlActions.motorControl.claw.setPosition(0.7); return false;})
                .endTrajectory()
                // GOTO BACKBOARD
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(-12, 55), Math.toRadians(0))
                .splineTo(new Vector2d(6, 55), Math.toRadians(0))
                .splineTo(new Vector2d(49.5,31), Math.toRadians(0))
                // PLACE PIXEL
                .stopAndAdd(new SequentialAction(
                        motorControlActions.placePixel(),
                        new SleepAction(0.5),
                        motorControlActions.pixelToHookCycle(),
                        new SleepAction(0.5),
                        motorControlActions.placePixel()
                ))
                // PARK
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(60,60, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    @Override
    public Action trajCenter(AprilTagDrive drive, MotorControlActions motorControlActions) {
        return null;
    }

    @Override
    public Action trajRight(AprilTagDrive drive, MotorControlActions motorControlActions) {
        return null;
    }
}
