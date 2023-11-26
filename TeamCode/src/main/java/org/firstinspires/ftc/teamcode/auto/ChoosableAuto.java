package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.experiments.AprilTagDrive;
import org.firstinspires.ftc.teamcode.motor.MotorControlActions;

public abstract class ChoosableAuto extends AbstractVisionOpMode {
    public abstract AbstractVisionOpMode parkClosePixelPlaceLeftAuto();
    public abstract AbstractVisionOpMode parkClosePixelPlaceRightAuto();
    public abstract AbstractVisionOpMode parkFarPixelPlaceLeftAuto();
    public abstract AbstractVisionOpMode parkFarPixelPlaceRightAuto();
    @Override
    public boolean choosable() {return true;}



    @Override
    public Action trajLeft(AprilTagDrive drive, MotorControlActions motorControlActions) {
        if (parkClose) {
            if (pixelPlaceLeft) {
                return parkClosePixelPlaceLeftAuto().trajLeft(drive, motorControlActions);
            } else {
                return parkClosePixelPlaceRightAuto().trajLeft(drive, motorControlActions);
            }
        } else {
            if (pixelPlaceLeft) {
                return parkFarPixelPlaceLeftAuto().trajLeft(drive, motorControlActions);
            } else {
                return parkFarPixelPlaceRightAuto().trajLeft(drive, motorControlActions);
            }
        }
    }

    @Override
    public Action trajCenter(AprilTagDrive drive, MotorControlActions motorControlActions) {
        if (parkClose) {
            if (pixelPlaceLeft) {
                return parkClosePixelPlaceLeftAuto().trajCenter(drive, motorControlActions);
            } else {
                return parkClosePixelPlaceRightAuto().trajCenter(drive, motorControlActions);
            }
        } else {
            if (pixelPlaceLeft) {
                return parkFarPixelPlaceLeftAuto().trajCenter(drive, motorControlActions);
            } else {
                return parkFarPixelPlaceRightAuto().trajCenter(drive, motorControlActions);
            }
        }
    }

    @Override
    public Action trajRight(AprilTagDrive drive, MotorControlActions motorControlActions) {
        if (parkClose) {
            if (pixelPlaceLeft) {
                return parkClosePixelPlaceLeftAuto().trajRight(drive, motorControlActions);
            } else {
                return parkClosePixelPlaceRightAuto().trajRight(drive, motorControlActions);
            }
        } else {
            if (pixelPlaceLeft) {
                return parkFarPixelPlaceLeftAuto().trajRight(drive, motorControlActions);
            } else {
                return parkFarPixelPlaceRightAuto().trajRight(drive, motorControlActions);
            }
        }
    }
}
