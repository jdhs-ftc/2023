/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auto.red;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.auto.AbstractVisionOpMode;
import org.firstinspires.ftc.teamcode.motor.MotorControl;
import org.firstinspires.ftc.teamcode.motor.MotorControlActions;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
@Autonomous(group = "Red", name = "LEFT Red Vision Auto", preselectTeleOp = "TeleopFieldCentric")
@Disabled // TODO: REMOVE AFTER CODING
public class LeftRedVisionAuto extends AbstractVisionOpMode
{

    @Override
    public PoseStorage.Team team() {
        return PoseStorage.Team.RED;
    }

    @Override
    public Pose2d startPose() {
        return new Pose2d(12,-63, Math.toRadians(90));
    }

    @Override
    public Action trajLeft(MecanumDrive drive, MotorControlActions motorControlActions) {
        return drive.actionBuilder(drive.pose)
                .strafeToSplineHeading(new Vector2d(12,-35), Rotation2d.exp(Math.toRadians(0)))
                .strafeToConstantHeading(new Vector2d(15,-35))
                .stopAndAdd(new SequentialAction(
                        motorControlActions.setCurrentMode(MotorControl.combinedMode.GRAB),
                        new SleepAction(0.25),
                        motorControlActions.lowerClaw.release(),
                        new SleepAction(0.1),
                        motorControlActions.setCurrentMode(MotorControl.combinedMode.IDLE)
                ))
                .strafeTo(new Vector2d(12, -35))
                .strafeTo(new Vector2d(12, -36))
                .splineToSplineHeading(new Pose2d(60,-60,Math.toRadians(-180.0000001)), Math.toRadians(0))
                .build();
    }

    @Override
    public Action trajCenter(MecanumDrive drive, MotorControlActions motorControlActions) {
        return drive.actionBuilder(drive.pose)
                .strafeToSplineHeading(new Vector2d(12,-35), Rotation2d.exp(Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(12,-32))
                .stopAndAdd(new SequentialAction(
                        motorControlActions.setCurrentMode(MotorControl.combinedMode.GRAB),
                        new SleepAction(0.25),
                        motorControlActions.lowerClaw.release(),
                        new SleepAction(0.1),
                        motorControlActions.setCurrentMode(MotorControl.combinedMode.IDLE)
                ))
                .strafeTo(new Vector2d(12, -36))
                .splineToSplineHeading(new Pose2d(60,-60,Math.toRadians(-180.0000001)), Math.toRadians(0))
                .build();
    }

    @Override
    public Action trajRight(MecanumDrive drive, MotorControlActions motorControlActions) {
        return drive.actionBuilder(drive.pose)
                .strafeToSplineHeading(new Vector2d(12,-33), Math.toRadians(-180))
                .strafeToConstantHeading(new Vector2d(10,-33))
                .stopAndAdd(new SequentialAction(
                        motorControlActions.setCurrentMode(MotorControl.combinedMode.GRAB),
                        new SleepAction(0.1),
                        motorControlActions.lowerClaw.release(),
                        new SleepAction(0.1),
                        motorControlActions.setCurrentMode(MotorControl.combinedMode.IDLE)
                ))
                .strafeTo(new Vector2d(12, -33))
                .splineToSplineHeading(new Pose2d(60,-60,Math.toRadians(-180.0000001)), Math.toRadians(0))
                .build();
    }
}