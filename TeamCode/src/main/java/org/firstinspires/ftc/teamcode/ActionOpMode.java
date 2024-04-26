package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public abstract class ActionOpMode extends LinearOpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private List<UniqueAction> uniqueActionsQueue = new ArrayList<>();

    protected void runBlocking(Action a) {
        Canvas c = new Canvas();
        a.preview(c);

        boolean b = true;
        while (b && !isStopRequested()) {
            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());

            b = a.run(p);

            dash.sendTelemetryPacket(p);
        }
    }

    protected void updateAsync(TelemetryPacket packet) {
        updateUniqueQueue();
        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;
    }

    private void updateUniqueQueue() {
        List<UniqueAction> oldActions = uniqueActionsQueue;
        uniqueActionsQueue = new ArrayList<>();
        oldActions.forEach(this::run);
    }

    protected void run(Action a) {
        if (a instanceof UniqueAction &&
                runningActions.stream().anyMatch(
                        (b) -> b instanceof UniqueAction &&
                                Objects.equals(((UniqueAction) b).key, ((UniqueAction) a).key))) {

            uniqueActionsQueue.add((UniqueAction) a);
        } else {
            runningActions.add(a);
        }
    }


}
