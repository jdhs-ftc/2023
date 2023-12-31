package com.example.meepmeeptesting;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;

/**
 * Dummy version of MotorActions for use with MeepMeep
 * This doesn't actually do anything, but it's here so that you don't have to comment it out in your trajectories
 */
public class MotorActions {
    Claw claw;
    AutoPlacer autoPlacer;

    public MotorActions() {
        this.claw = new Claw();
        this.autoPlacer = new AutoPlacer();
    }



    public SleepAction pixelToHook() {
        return new SleepAction(1.75);
    }

    public SleepAction placePixel() {
        return new SleepAction(1);
    }

    public SleepAction autoPlace() {
        return new SleepAction(0.5);
    }



    public class Claw {
        public SleepAction grab() {
            return new SleepAction(0.4);
        }

        // release
        public SleepAction release() {
            return new SleepAction(0.4);
        }
    }

    public class AutoPlacer {
        public Action place() {
            return new Action() {
                @Override
                public boolean run(TelemetryPacket t) {

                    return false;
                }
            };
        }
    }
}
