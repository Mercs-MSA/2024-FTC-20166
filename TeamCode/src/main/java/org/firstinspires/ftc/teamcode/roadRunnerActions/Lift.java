//package org.firstinspires.ftc.teamcode.roadRunnerActions;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class Lift {
//    private DcMotor liftMotor = null;
//
//    public Lift(HardwareMap hardwareMap) {
//        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
//        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public class LiftExtend implements Action {
//        private boolean initialized = false;
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!initialized) {
//                liftMotor.setTargetPosition(100);
//                initialized = true;
//            }
//            return true;
//        }
//    }
//
//    public Action liftExtend() {
//        return new LiftExtend();
//    }
//}
