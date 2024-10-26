package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubSystemIntakeSlide {
    private DcMotorEx intakeSlide;
    private static final int MAX_EXTENSION_COUNT = 1150;
    private static final int MIN_EXTENSION_COUNT = 0;


    public SubSystemIntakeSlide(HardwareMap hardwareMap) throws InterruptedException {                 // Motor Mapping
        // Initialize the motor hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        intakeSlide = hardwareMap.get(DcMotorEx.class, "intakeSlide");       //Sets the names of the hardware on the hardware map
        intakeSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeSlide.setTargetPosition(0);
        intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlide.setPower(0.7);
    }


    public void setPosition(int encoderPosition) {
        intakeSlide.setTargetPosition(encoderPosition);
    }

    public void movePosition(double inspeed, double outspeed) {
        if (inspeed > 0.5)
            setPosition(MAX_EXTENSION_COUNT);
        else if (outspeed > 0.5)
            setPosition(MIN_EXTENSION_COUNT);
        else
            setPosition(getPosition());

    }

    public int getPosition() {
        return intakeSlide.getCurrentPosition();
    }
}