package org.firstinspires.ftc.teamcode.helpers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 * <p>
 * Also we store the team here so we can use it in the vision code and for determining forward regarding field centric
 */
@Config
public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(0,0,Math.toRadians(-90));
    public static Team currentTeam = Team.BLUE;
    public enum Team {
        BLUE, RED
    }
}
