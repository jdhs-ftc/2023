package org.firstinspires.ftc.teamcode.motor;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class MotorControlActions {
    private final MotorControl motorControl;
    public final Slide slide;
    public final Arm arm;

    public MotorControlActions(MotorControl motorControl) {
        this.motorControl = motorControl;
        this.slide = new Slide();
        this.arm = new Arm();
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
                return true;
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
                return true;
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
                return true;
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
                    return true;
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
                    return true;
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
                    return true;
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
                    return true;
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
}
