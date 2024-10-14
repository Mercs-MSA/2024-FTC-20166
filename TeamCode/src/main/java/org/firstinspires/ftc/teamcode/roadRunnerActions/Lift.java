package org.firstinspires.ftc.teamcode.roadRunnerActions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.teamcode.Constants.slideTickPerIn;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotor liftMotor = null;

    public Lift(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public class LiftHighBasket implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftMotor.setPower(.5);
                liftMotor.setTargetPosition((int)(31 * slideTickPerIn));
                initialized = true;
            }
            return true;
        }
    }

    public Action liftHighBasket() {
        return new LiftHighBasket();
    }

    public class LiftHighSpec implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {}
            return true;
        }
    }
}
