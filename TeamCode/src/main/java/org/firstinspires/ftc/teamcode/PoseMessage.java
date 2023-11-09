package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;

public final class PoseMessage {
    public final long timestamp;
    public final double x;
    public final double y;
    public final double heading;

    public PoseMessage(Pose2d pose) {
        this.timestamp = System.nanoTime();
        this.x = pose.position.x;
        this.y = pose.position.y;
        this.heading = pose.heading.log();
    }

    @NonNull
    @Override
    public String toString() {
        return "PoseMessage{" +
                "timestamp=" + timestamp +
                ", x=" + x +
                ", y=" + y +
                ", heading=" + heading +
                '}';
    }
}

