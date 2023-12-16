package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);
        MotorActions motorActions = new MotorActions();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueLight())
                .setDimensions(14,17.25)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, -62, Math.toRadians(-90)))
                .setReversed(true)
                // GOTO GROUND PIXEL
                .splineToConstantHeading(new Vector2d(-49,-21), Math.toRadians(90))
                .endTrajectory()
                .stopAndAdd(motorActions.claw.release())

                // GOTO STACK
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-62,-12,Math.toRadians(180)), Math.toRadians(180))
                .endTrajectory()
                .stopAndAdd(motorActions.claw.grab())
                .waitSeconds(0.5)


                // GOTO BACKBOARD
                .setReversed(true)
                .afterTime(0.5, motorActions.pixelToHook())
                .splineToConstantHeading(new Vector2d(-36, -14), Math.toRadians(0))
                .splineTo(new Vector2d(20, -14), Math.toRadians(0))
                //.afterTime(0.5, drive.CorrectWithTagAction())
                .splineToSplineHeading(new Pose2d(56.5,-30, Math.toRadians(180)), Math.toRadians(0))
                .stopAndAdd(new ParallelAction(motorActions.autoPlace(),motorActions.placePixel()))
                .endTrajectory()


                // PARK
                .splineToLinearHeading(new Pose2d(54,-14, Math.toRadians(180)), Math.toRadians(0))
                .build());

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14,17.25)
                .setColorScheme(new ColorSchemeRedLight())

                .build();

        myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(12, 62, Math.toRadians(90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(8,32, Math.toRadians(180)), Math.toRadians(-90))
                .endTrajectory()
                /*
                .stopAndAdd(drive.CorrectWithTagAction())
                .stopAndAdd(telemetryPacket -> {
                    motorActions.motorControl.claw.setPosition(0.95);
                    return false;
                })

                 */
                // PLACE HERE
                .strafeTo(new Vector2d(9,32))
                .splineTo(new Vector2d(55.75,29), Math.toRadians(0))
                /*
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {motorActions.motorControl.autoPlacer.setPosition(0.5); return false;},
                        new SleepAction(0.5),
                        telemetryPacket -> {motorActions.motorControl.autoPlacer.setPosition(1); return false;}))

                 */
                        .endTrajectory()
                        .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(60,60, Math.toRadians(180)), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .start();


    }
}