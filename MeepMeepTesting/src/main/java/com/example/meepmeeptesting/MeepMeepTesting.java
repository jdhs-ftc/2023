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
                .setConstraints(40, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14,17.25)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, 62, Math.toRadians(90)))
                .setReversed(true)
                // GOTO GROUND PIXEL
                .splineToSplineHeading(new Pose2d(-46,21, Math.toRadians(90)), Math.toRadians(-90))
                .endTrajectory()
                /*
                .stopAndAdd(drive.CorrectWithTagAction())
                .stopAndAdd(telemetryPacket -> {
                    motorControlActions.motorControl.claw.setPosition(0.95);
                    return false;
                })

                 */
                // GOTO BACKBOARD
                .setReversed(true)
                .splineTo(new Vector2d(-24, 18), Math.toRadians(0))
                .splineTo(new Vector2d(20, 18), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(50.5,40, Math.toRadians(180)), Math.toRadians(0))
                /*
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {motorControlActions.motorControl.autoPlacer.setPosition(0.5); return false;},
                        new SleepAction(0.5),
                        telemetryPacket -> {motorControlActions.motorControl.autoPlacer.setPosition(1); return false;}))

                 */

                .endTrajectory()


                // PARK
                .splineToLinearHeading(new Pose2d(54,13, Math.toRadians(180)), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();


    }
}