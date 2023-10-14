package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.motor.MotorControl;
import org.firstinspires.ftc.teamcode.motor.MotorControlActions;
import org.firstinspires.ftc.teamcode.motor.MotorControlActions.RaceParallelCommand;

@Autonomous(name = "TestAutoOpMode", group = "Test")
@Disabled
public class TestAutoOpMode extends ActionOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        MotorControl motorControl = new MotorControl(hardwareMap);
        MotorControlActions motorControlActions = new MotorControlActions(motorControl);


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Action traj =
                drive.actionBuilder(drive.pose)
                .stopAndAdd(motorControlActions.setCurrentMode(MotorControl.combinedPreset.IDLE))
                .splineTo(new Vector2d(0, 30), Math.PI / 2)
                .stopAndAdd(motorControlActions.slide.setTargetPosition(300))
                .splineTo(new Vector2d(0, 60), Math.PI)
                .stopAndAdd(motorControlActions.setCurrentMode(MotorControl.combinedPreset.PLACE))

                .stopAndAdd(motorControlActions.waitUntilFinished())
                        .stopAndAdd(motorControlActions.setCurrentMode(MotorControl.combinedPreset.GRAB))
                        .stopAndAdd(motorControlActions.waitUntilFinished())
                .build();

        waitForStart();

        if (isStopRequested()) return;

        runBlocking(new RaceParallelCommand(
                traj,
                motorControlActions.update()
        ));



        PoseStorage.currentPose = drive.pose;
    }

}
