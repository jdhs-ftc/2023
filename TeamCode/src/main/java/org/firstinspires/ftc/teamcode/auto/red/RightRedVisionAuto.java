package org.firstinspires.ftc.teamcode.auto.red;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.auto.AbstractVisionOpMode;
import org.firstinspires.ftc.teamcode.motor.MotorControl;
import org.firstinspires.ftc.teamcode.motor.MotorControlActions;
@Autonomous(group = "Blue", name = "LEFT Blue Vision Auto", preselectTeleOp = "TeleopFieldCentric")
public class RightRedVisionAuto extends AbstractVisionOpMode {
    @Override
    public PoseStorage.Team team() {
        return PoseStorage.Team.RED;
    }

    @Override
    public Pose2d startPose() {
        return new Pose2d(-36,63, Math.toRadians(-90));
    }
    @Override
    public Action trajLeft(MecanumDrive drive, MotorControlActions motorControlActions) {
        return drive.actionBuilder(drive.pose)
                .strafeToSplineHeading(new Vector2d(-36,35), Rotation2d.exp(Math.toRadians(0)))
                .strafeToConstantHeading(new Vector2d(-33,35))
                .stopAndAdd(new SequentialAction(
                        motorControlActions.setCurrentMode(MotorControl.combinedMode.GRAB),
                        new SleepAction(0.25),
                        motorControlActions.lowerClaw.release(),
                        new SleepAction(0.1),
                        motorControlActions.setCurrentMode(MotorControl.combinedMode.IDLE)
                ))
                .strafeTo(new Vector2d(-36, 35))
                .strafeTo(new Vector2d(-36, 12))
                .strafeTo(new Vector2d(-35, 12))
                .splineTo(new Vector2d(12, 12), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(60,12,Math.toRadians(180.0000001)), Math.toRadians(0))
                .build();
    }

    @Override
    public Action trajCenter(MecanumDrive drive, MotorControlActions motorControlActions) {
        return drive.actionBuilder(drive.pose)
                .strafeToSplineHeading(new Vector2d(-36,35), Rotation2d.exp(Math.toRadians(-90)))
                .endTrajectory()
                .strafeToConstantHeading(new Vector2d(-36,32))
                .stopAndAdd(new SequentialAction(
                        motorControlActions.setCurrentMode(MotorControl.combinedMode.GRAB),
                        new SleepAction(0.25),
                        motorControlActions.lowerClaw.release(),
                        new SleepAction(0.1),
                        motorControlActions.setCurrentMode(MotorControl.combinedMode.IDLE)
                ))
                .strafeTo(new Vector2d(-36, 35))
                .strafeTo(new Vector2d(-55, 35))
                .strafeTo(new Vector2d(-55, 30))
                .splineTo(new Vector2d(12, 12), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(60,12,Math.toRadians(180.0000001)), Math.toRadians(0))
                .build();
    }

    @Override
    public Action trajRight(MecanumDrive drive, MotorControlActions motorControlActions) {
        return drive.actionBuilder(drive.pose)
                .strafeToSplineHeading(new Vector2d(-36,35), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(-39,35))
                .stopAndAdd(new SequentialAction(
                        motorControlActions.setCurrentMode(MotorControl.combinedMode.GRAB),
                        new SleepAction(0.25),
                        motorControlActions.lowerClaw.release(),
                        new SleepAction(0.1),
                        motorControlActions.setCurrentMode(MotorControl.combinedMode.IDLE)
                ))
                .strafeTo(new Vector2d(-36, 35))
                .strafeTo(new Vector2d(-36, 12))
                .strafeTo(new Vector2d(-35, 12))
                .splineTo(new Vector2d(12, 12), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(60,12,Math.toRadians(180.0000001)), Math.toRadians(0))
                .build();
    }
}
