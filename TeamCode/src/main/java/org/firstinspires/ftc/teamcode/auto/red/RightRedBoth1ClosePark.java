package org.firstinspires.ftc.teamcode.auto.red;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.auto.AbstractVisionOpMode;
import org.firstinspires.ftc.teamcode.motor.MotorControlActions;
@Autonomous
public class RightRedBoth1ClosePark extends AbstractVisionOpMode {
    /**
     * Is this a red or a blue autonomous?
     *
     * @return the team
     */
    @Override
    public PoseStorage.Team team() {
        return PoseStorage.Team.RED;
    }

    /**
     * Starting Position of the trajectories
     *
     * @return the starting pose
     */
    @Override
    public Pose2d startPose() {
        return new Pose2d(12,-60,Math.toRadians(90));
    }

    @Override
    public Action trajLeft(MecanumDrive drive, MotorControlActions motorControlActions) {
        return null;
    }

    @Override
    public Action trajCenter(MecanumDrive drive, MotorControlActions motorControlActions) {
        return null;
    }

    @Override
    public Action trajRight(MecanumDrive drive, MotorControlActions motorControlActions) {
        return null;
    }
}
