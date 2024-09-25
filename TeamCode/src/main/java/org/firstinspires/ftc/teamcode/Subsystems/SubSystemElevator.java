package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubSystemElevator {
    // Instantiate the drivetrain motor variables
    private DcMotorEx elevator;


    public SubSystemElevator(HardwareMap hardwareMap) throws InterruptedException {                 // Motor Mapping
        // Initialize the motor hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        elevator = hardwareMap.get(DcMotorEx.class, "elevator");       //Sets the names of the hardware on the hardware map
        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setPower(1);
    }

    public void setPosition(int encoderPosition) {
        elevator.setTargetPosition(encoderPosition);
    }

    public int getPosition() {
        return elevator.getCurrentPosition();
    }
}