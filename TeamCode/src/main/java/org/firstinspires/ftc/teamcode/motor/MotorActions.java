package org.firstinspires.ftc.teamcode.motor;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

public class MotorActions {
    public final MotorControl motorControl;
    public final Slide slide;
    public final ClawArm clawArm;
    public final Claw claw;
    public final AutoPlacer autoPlacer;
    public final Hook hook;
    public final Seperator seperator;

    public MotorActions(MotorControl motorControl) {
        this.motorControl = motorControl;
        this.slide = new Slide();
        this.clawArm = new ClawArm();
        this.claw = new Claw();
        this.hook = new Hook();
        this.autoPlacer = new AutoPlacer();
        this.seperator = new Seperator();
    }
    public Action waitUntilFinished() {
        return t -> motorControl.closeEnough();
    }

    public Action update() {
        return t -> {
            motorControl.update();
            return true; // this returns true to make it loop forever; use RaceParallelCommand
        };
    }

    public Action pixelToHook() {
        return new SequentialAction(
                slide.moveDown(),
                clawArm.moveHook(),
                clawArm.waitUntilFinished(),
                claw.release(),
                seperator.hold(),
                clawArm.moveDown(),
                clawArm.waitUntilFinished()
        );
    }

    public Action placePixel() {
        return new SequentialAction(
                hookToBackdrop(),
                new SleepAction(0.6),
                returnHook()
        );
    }

    public Action placeTwoPixel() {
        return new SequentialAction(
                hookToBackdrop(),
                new SleepAction(0.6),
                placeSecondPixel(),
                new SleepAction(0.3),
                returnHook()
        );
    }

    public Action placeSecondPixel() {
        return new SequentialAction(
                hook.seperatePos(),
                new SleepAction(0.5),
                seperator.release(),
                hook.raise(),
                seperator.release()
        );
    }

    public Action hookToBackdrop() {
        return new SequentialAction(
                slide.moveUp(),
                new SleepAction(0.4),
                hook.raise()
        );
    }

    public Action returnHook() {
        return new SequentialAction(
                hook.lower(),
                slide.moveDown()
        );
    }

    public Action log(String message) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket t) {
                System.out.println(message);
                return false;
            }
        };
    }

    public Action autoPlace() {
        return autoPlacer.place();
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
                    return !motorControl.clawArm.closeEnough();
                }
            };
        }

        public Action moveHook() {
            return (telemetryPacket -> {motorControl.clawArm.moveToHook();return false;});
        }
        public Action moveDown() {
            return (telemetryPacket -> {motorControl.clawArm.moveDown();return false;});
        }
    }
    public class Slide {
        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.slide.setTargetPosition(position);
                return false;
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.slide.closeEnough();
                }
            };
        }

        public Action moveUp() {
            return setTargetPosition(1200);
        }
        public Action moveDown() {
            return setTargetPosition(40);
        }
    }

    public class Claw {
        public Action grab() {
            return new SequentialAction(t -> {motorControl.claw.setPosition(0.8);return false;},
                    new SleepAction(0.4));
        }


        // release
        public Action release() {
            return new SequentialAction(t -> {motorControl.claw.setPosition(0.91);return false;},new SleepAction(0.4));
        }
    }
    public class Hook {
        public Action raise() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.hookArm.setPosition(0.5);
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
        public Action seperatePos(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.hookArm.setPosition(0.7); // TODO TUNE
                    return false;
                }
            };
        }
        public Action goToPos(double pos){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.hookArm.setPosition(pos);
                    return false;
                }
            };
        }
    }

    public class AutoPlacer {
        public Action place() {
            return new SequentialAction(
                    telemetryPacket -> {motorControl.autoPlacer.setPosition(0.3); return false;},
                    new SleepAction(0.75),
                    telemetryPacket -> {motorControl.autoPlacer.setPosition(1); return false;});
        }
    }

    public class Seperator {
        public Action hold() {
            return new InstantAction(() -> motorControl.seperator.setPosition(0.25)); // TODO TUNE
        }
        public Action release() {
            return new InstantAction(() -> motorControl.seperator.setPosition(0));
        }
    }


}
