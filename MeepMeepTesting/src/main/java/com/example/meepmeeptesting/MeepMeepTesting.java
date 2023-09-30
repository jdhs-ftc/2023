package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14,18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(12,33), Rotation2d.exp(Math.toRadians(180)))
                .strafeToConstantHeading(new Vector2d(10,33))
                //.stopAndAdd(motorControlActions.setCurrentMode(MotorControl.combinedMode.GRAB))
                .waitSeconds(0.25)
                //.stopAndAdd(motorControlActions.lowerClaw.release()) // place first pixel
                .endTrajectory()
                .strafeTo(new Vector2d(12, 33))
                //.stopAndAdd(motorControlActions.setCurrentMode(MotorControl.combinedMode.PLACE)) // raise arm/slide to prepare to place pixel
                .strafeTo(new Vector2d(12, 50))
                .strafeTo(new Vector2d(15,50))
                .splineToSplineHeading(new Pose2d(48, 30,Math.toRadians(0)), Math.toRadians(0))
                .strafeTo(new Vector2d(50, 30))
                .waitSeconds(1)
                .endTrajectory()
                //.stopAndAdd(motorControlActions.upperClaw.release()) // place pixel
                .strafeTo(new Vector2d(48, 30))
                .splineToLinearHeading(new Pose2d(60,60, Math.toRadians(180)), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}