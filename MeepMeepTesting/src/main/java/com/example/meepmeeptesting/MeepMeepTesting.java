package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14,18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, 63, Math.toRadians(90)))
                .stopAndAdd(new SleepAction(1))
                .strafeToSplineHeading(new Vector2d(12,35), Rotation2d.exp(Math.toRadians(0)))

                /*
                .stopAndAdd(new SequentialAction(
                        motorControlActions.setCurrentMode(MotorControl.combinedPreset.GRAB),
                        new SleepAction(0.1),
                        motorControlActions.lowerClaw.release(),
                        new SleepAction(0.25),
                        motorControlActions.slide.setTargetPosition(150)
                ))*/
                .strafeTo(new Vector2d(11, 35))
                .strafeTo(new Vector2d(11, 34))
                .splineToConstantHeading(new Vector2d(10, 12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50,35), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}