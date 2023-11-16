package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14,18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, 63, Math.toRadians(90)))
                .setReversed(true)
                // GOTO GROUND PIXEL
                .splineToSplineHeading(new Pose2d(32,33, Math.toRadians(180)), Math.toRadians(0))
                .endTrajectory()
                /*
                .stopAndAdd(telemetryPacket -> {
                    motorControlActions.motorControl.claw.setPosition(0.95);
                    return false;
                })

                 */
                // GOTO BACKBOARD
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(50.5,37, Math.toRadians(180)), Math.toRadians(0))
                /*
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {motorControlActions.motorControl.autoPlacer.setPosition(0.5); return false;},
                        new SleepAction(0.5),
                        telemetryPacket -> {motorControlActions.motorControl.autoPlacer.setPosition(1); return false;}))

                 */
                .endTrajectory()

                // STACK
                .splineTo(new Vector2d(-12, 60), Math.toRadians(180))
                .splineTo(new Vector2d(-61,35), Math.toRadians(180))
                // GRAB PIXEL
                .endTrajectory()
                .strafeTo(new Vector2d(-56,35))
                // PIXEL TO HOOK
                .endTrajectory()
                .strafeTo(new Vector2d(-61,35))
                // GRAB PIXEL
                .endTrajectory()
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(-12, 60), Math.toRadians(0))
                .splineTo(new Vector2d(6, 60), Math.toRadians(0))
                .splineTo(new Vector2d(50.5,31), Math.toRadians(0))
                // PLACE PIXEL

                // PARK
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(60,60, Math.toRadians(180)), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();


    }
}