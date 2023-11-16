package org.firstinspires.ftc.teamcode.motor;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

public class MotorControlActions {
    public final MotorControl motorControl;
    public final Slide slide;
    public final Arm arm;
    public final LowerClaw lowerClaw;
    public final UpperClaw upperClaw;

    public MotorControlActions(MotorControl motorControl) {
        this.motorControl = motorControl;
        this.slide = new Slide();
        this.arm = new Arm();
        this.lowerClaw = new LowerClaw();
        this.upperClaw = new UpperClaw();
    }




    public Action waitUntilFinished() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket t) {
                return motorControl.closeEnough();
            }

            @Override
            public void preview(@NonNull Canvas canvas) {

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

            @Override
            public void preview(@NonNull Canvas canvas) {

            }
        };
    }

    public Action pixelToHookCycle() { // TODO TUNE
        return new SequentialAction(
                telemetryPacket -> {motorControl.clawArm.moveToHook(); return false;},
                new SleepAction(1),
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

    public class Arm {
        public Action reset() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.clawArm.reset();
                    return false;
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

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

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }

        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return motorControl.clawArm.closeEnough();
                }

                @Override
                public void preview(@NonNull Canvas canvas) {


                }
            };
        }
    }
    public class Slide {
        public Action reset() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.slide.reset();
                    return false;
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }

        public Action setTargetPosition(double position) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.slide.setTargetPosition(position);
                    return false;
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return motorControl.slide.closeEnough();
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }
    }

    public class LowerClaw {
        public Action grab() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.claw.setPosition(1);
                    return false;
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }

        // release
        public Action release() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.claw.setPosition(0);
                    return false;
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }
    }
    public class UpperClaw {
        public Action grab() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.hookArm.setPosition(1);
                    return false;
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }

        // release
        public Action release() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.hookArm.setPosition(0);
                    return false;
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }
    }
}
