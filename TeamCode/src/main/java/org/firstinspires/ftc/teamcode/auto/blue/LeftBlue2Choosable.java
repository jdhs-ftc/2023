package org.firstinspires.ftc.teamcode.auto.blue;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.auto.AbstractVisionOpMode;
import org.firstinspires.ftc.teamcode.auto.ChoosableAuto;
import org.firstinspires.ftc.teamcode.auto.blue.LeftBlue2.CloseParkLeftPixel;
import org.firstinspires.ftc.teamcode.auto.blue.LeftBlue2.FarParkLeftPixel;

@Autonomous(preselectTeleOp = "Teleop Field Centric")
public class LeftBlue2Choosable extends ChoosableAuto {
    /**
     * Is this a red or a blue autonomous?
     *
     * @return the team
     */
    @Override
    public PoseStorage.Team team() {
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
    public AbstractVisionOpMode parkClosePixelPlaceLeftAuto() {
        return new CloseParkLeftPixel();
    }

    @Override
    public AbstractVisionOpMode parkClosePixelPlaceRightAuto() {
        return null;
    }

    @Override
    public AbstractVisionOpMode parkFarPixelPlaceLeftAuto() {
        return new FarParkLeftPixel();
    }

    @Override
    public AbstractVisionOpMode parkFarPixelPlaceRightAuto() {
        return null;
    }
}
