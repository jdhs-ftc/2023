package com.example.meepmeeptesting;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

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



    public Action pixelToHook() {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket t) {

                return false;
            }
        };
    }

    public Action placePixel() {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket t) {

                return false;
            }
        };
    }

    public Action autoPlace() {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket t) {

                return false;
            }
        };
    }



    public class Claw {
        public Action grab() {
            return new Action() {
                @Override
                public boolean run(TelemetryPacket t) {

                    return false;
                }
            };
        }

        // release
        public Action release() {
            return new Action() {
                @Override
                public boolean run(TelemetryPacket t) {

                    return false;
                }
            };
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
