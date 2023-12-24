package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.experimentsSemiBroken.AprilTagDrive;
import org.firstinspires.ftc.teamcode.motor.MotorActions;

/**
 * I was experimenting here with using telemetry actions to select an auto without getting lost
 * in the opmodes menu.
 * However the telemetry actions thing is for updating telemetry as it's displayed not for making
 * it interactable
 * <p>
 * <b>This doesn't do anything</b>,( might be useful later), the real auto code is AbstractVisionOpMode and the implementing classes
 */
@Deprecated
public abstract class ChoosableAuto extends AbstractVisionOpMode {
    public abstract AbstractVisionOpMode parkClosePixelPlaceLeftAuto();
    public abstract AbstractVisionOpMode parkClosePixelPlaceRightAuto();
    public abstract AbstractVisionOpMode parkFarPixelPlaceLeftAuto();
    public abstract AbstractVisionOpMode parkFarPixelPlaceRightAuto();
    @Override
    public boolean choosable() {return true;}



    @Override
    public Action trajLeft(AprilTagDrive drive, MotorActions motorActions) {
        if (parkClose) {
            if (pixelPlaceLeft) {
                return parkClosePixelPlaceLeftAuto().trajLeft(drive, motorActions);
            } else {
                return parkClosePixelPlaceRightAuto().trajLeft(drive, motorActions);
            }
        } else {
            if (pixelPlaceLeft) {
                return parkFarPixelPlaceLeftAuto().trajLeft(drive, motorActions);
            } else {
                return parkFarPixelPlaceRightAuto().trajLeft(drive, motorActions);
            }
        }
    }

    @Override
    public Action trajCenter(AprilTagDrive drive, MotorActions motorActions) {
        if (parkClose) {
            if (pixelPlaceLeft) {
                return parkClosePixelPlaceLeftAuto().trajCenter(drive, motorActions);
            } else {
                return parkClosePixelPlaceRightAuto().trajCenter(drive, motorActions);
            }
        } else {
            if (pixelPlaceLeft) {
                return parkFarPixelPlaceLeftAuto().trajCenter(drive, motorActions);
            } else {
                return parkFarPixelPlaceRightAuto().trajCenter(drive, motorActions);
            }
        }
    }

    @Override
    public Action trajRight(AprilTagDrive drive, MotorActions motorActions) {
        if (parkClose) {
            if (pixelPlaceLeft) {
                return parkClosePixelPlaceLeftAuto().trajRight(drive, motorActions);
            } else {
                return parkClosePixelPlaceRightAuto().trajRight(drive, motorActions);
            }
        } else {
            if (pixelPlaceLeft) {
                return parkFarPixelPlaceLeftAuto().trajRight(drive, motorActions);
            } else {
                return parkFarPixelPlaceRightAuto().trajRight(drive, motorActions);
            }
        }
    }
}
