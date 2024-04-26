package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class UniqueAction implements Action {
    public String key = "UniqueAction";
    private final Action action;
    public UniqueAction(String key, Action action) {
        this.key = key;
        this.action = action;
    }
    public UniqueAction(Action action) {
        this.action = action;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket t) {
        return action.run(t);
    }

    @Override
    public void preview(@NonNull Canvas fieldOverlay) {
        action.preview(fieldOverlay);
    }
}
