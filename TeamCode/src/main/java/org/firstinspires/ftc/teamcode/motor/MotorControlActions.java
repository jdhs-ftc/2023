package org.firstinspires.ftc.teamcode.motor;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class MotorControlActions {
    private final MotorControl motorControl;
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

    public static class RaceParallelCommand implements Action {
        private final Action[] actions;

        public RaceParallelCommand(Action... actions) {
            this.actions = actions;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket t) {
            boolean finished = true;
            for (Action action : actions) finished = finished && action.run(t);
            return finished;
        }

        @Override
        public void preview(@NonNull Canvas canvas) {
            for (Action action : actions) action.preview(canvas);
        }


    }



    public Action setCurrentMode(MotorControl.combinedMode newMode) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket t) {
                motorControl.setCurrentMode(newMode);
                return false;
            }

            @Override
            public void preview(@NonNull Canvas canvas) {

            }
        };
    }
    public Action reset() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket t) {
                motorControl.reset();
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
                return motorControl.arm.isBusy() || motorControl.slide.isBusy();
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
                return false;
            }

            @Override
            public void preview(@NonNull Canvas canvas) {

            }
        };
    }

    public class Arm {
        public Action reset() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.arm.reset();
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
                    motorControl.arm.setTargetPosition(position);
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
                    return motorControl.arm.isBusy();
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
                    return motorControl.slide.isBusy();
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
                    motorControl.lowerClaw.setPower(1);
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
                    motorControl.lowerClaw.setPower(0);
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
                    motorControl.upperClaw.setPower(1);
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
                    motorControl.upperClaw.setPower(0);
                    return false;
                }

                @Override
                public void preview(@NonNull Canvas canvas) {

                }
            };
        }
    }
}
