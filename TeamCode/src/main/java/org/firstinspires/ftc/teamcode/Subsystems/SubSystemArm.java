package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SubSystemArm {
    // Instantiate the drivetrain motor variables
    private DcMotorEx arm;


    public SubSystemArm(HardwareMap hardwareMap) throws InterruptedException {                 // Motor Mapping
        // Initialize the motor hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        arm = hardwareMap.get(DcMotorEx.class, "arm");       //Sets the names of the hardware on the hardware map
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
    }

    public void setPosition(int encoderPosition) {
        arm.setTargetPosition(encoderPosition);
    }

    public int getPosition() {
        return arm.getCurrentPosition();
    }
}