package org.firstinspires.ftc.teamcode.auto.blue.LeftBlue2plus2;

import com.acmerobotics.roadrunner.Action;
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
@Autonomous(preselectTeleOp = "Teleop Field Centric", name = "Left Blue, 2+2, Close Park, Left Pixel", group = "Blue")
@Disabled
public class LeftBlue2plus2ClosePark extends AbstractVisionOpMode {
    /**
     * Is this a red or a blue autonomous?
     *
     * @return the team
     */
    @Override
    public PoseStorage.Team team() {
        this.getClass().getPackage().getName();
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
                .splineToSplineHeading(new Pose2d(28,31, Math.toRadians(180)), Math.toRadians(0))
                .endTrajectory()

                .stopAndAdd(telemetryPacket -> {
                    motorActions.motorControl.claw.setPosition(0.95);
                    return false;
                })


                // GOTO BACKBOARD
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(49.75,37, Math.toRadians(180)), Math.toRadians(0))

                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {motorActions.motorControl.autoPlacer.setPosition(0.5); return false;},
                        new SleepAction(0.5),
                        telemetryPacket -> {motorActions.motorControl.autoPlacer.setPosition(1); return false;}))


                .endTrajectory()

                // STACK
                .splineTo(new Vector2d(-12, 55), Math.toRadians(180))
                .splineTo(new Vector2d(-62,35), Math.toRadians(180))
                // GRAB PIXEL
                .stopAndAdd(telemetryPacket -> {motorActions.motorControl.claw.setPosition(0.7); return false;})
                .endTrajectory()
                .strafeTo(new Vector2d(-56,35))


                // CYCLE
                .stopAndAdd(motorActions.pixelToHook())
                .endTrajectory()
                .strafeTo(new Vector2d(-62,35))
                // GRAB PIXEL
                .stopAndAdd(telemetryPacket -> {motorActions.motorControl.claw.setPosition(0.7); return false;})
                .endTrajectory()
                // GOTO BACKBOARD
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(-12, 55), Math.toRadians(0))
                .splineTo(new Vector2d(6, 55), Math.toRadians(0))
                .splineTo(new Vector2d(49.5,31), Math.toRadians(0))
                // PLACE PIXEL
                .stopAndAdd(new SequentialAction(
                        motorActions.placePixel(),
                        new SleepAction(0.5),
                        motorActions.pixelToHook(),
                        new SleepAction(0.5),
                        motorActions.placePixel()
                ))
                // PARK
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(60,60, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    @Override
    public Action trajCenter(AprilTagDrive drive, MotorActions motorActions) {
        return null;
    }

    @Override
    public Action trajRight(AprilTagDrive drive, MotorActions motorActions) {
        return null;
    }
}
