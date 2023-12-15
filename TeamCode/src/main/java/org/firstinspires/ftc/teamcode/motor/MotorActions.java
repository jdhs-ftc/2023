package org.firstinspires.ftc.teamcode.motor;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

public class MotorActions {
    public final MotorControl motorControl;
    public final Slide slide;
    public final ClawArm clawArm;
    public final Claw claw;
    public final AutoPlacer autoPlacer;
    public final Hook hook;

    public MotorActions(MotorControl motorControl) {
        this.motorControl = motorControl;
        this.slide = new Slide();
        this.clawArm = new ClawArm();
        this.claw = new Claw();
        this.hook = new Hook();
        this.autoPlacer = new AutoPlacer();
    }
    public Action waitUntilFinished() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket t) {
                return motorControl.closeEnough();
            }
        };
    }

    public Action update() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket t) {
                motorControl.update();
                return true;
            }
        };
    }

    public Action pixelToHook() {
        return new SequentialAction(
                telemetryPacket -> {motorControl.clawArm.moveToHook(); return false;},
                new SleepAction(1.25),
                telemetryPacket -> { motorControl.claw.setPosition(0.95); return false;},
                new SleepAction(0.25),
                telemetryPacket -> {motorControl.clawArm.moveDown(); return false;},
                new SleepAction(0.25)
        );
    }

    public Action placePixel() {
        return new SequentialAction(
                telemetryPacket -> {motorControl.slide.setTargetPosition(1000); return false;},
                new SleepAction(0.4),
                telemetryPacket -> {motorControl.hookArm.setPosition(0.2); return false;},
                new SleepAction(0.6),
                telemetryPacket -> {motorControl.hookArm.setPosition(1); return false;},
                telemetryPacket -> {motorControl.slide.setTargetPosition(-40); return false;}
        );
    }

    public Action autoPlace() {
        return new SequentialAction(
                telemetryPacket -> {motorControl.autoPlacer.setPosition(0.3); return false;},
                new SleepAction(0.5),
                telemetryPacket -> {motorControl.autoPlacer.setPosition(1); return false;});
    }

    public class ClawArm {
        public Action reset() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.clawArm.reset();
                    return false;
                }
            };
        }

        public Action setTargetPosition(double position) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.clawArm.setTargetPosition(position);
                    return false;
                }
            };
        }

        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return motorControl.clawArm.closeEnough();
                }
            };
        }
    }
    public class Slide {
        public Action setTargetPosition(double position) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.slide.setTargetPosition(position);
                    return false;
                }
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return motorControl.slide.closeEnough();
                }
            };
        }
    }

    public class Claw {
        public Action grab() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.claw.setPosition(0.8);
                    return false;
                }
            };
        }

        // release
        public Action release() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.claw.setPosition(0.95);
                    return false;
                }
            };
        }
    }
    public class Hook {
        public Action raise() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.hookArm.setPosition(0.3);
                    return false;
                }
            };
        }
        public Action lower() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.hookArm.setPosition(1);
                    return false;
                }
            };
        }
    }

    public class AutoPlacer {
        public Action place() {
            return new SequentialAction(
                    telemetryPacket -> {motorControl.autoPlacer.setPosition(0.5); return false;},
                    new SleepAction(0.5),
                    telemetryPacket -> {motorControl.autoPlacer.setPosition(1); return false;});
        }
    }
}