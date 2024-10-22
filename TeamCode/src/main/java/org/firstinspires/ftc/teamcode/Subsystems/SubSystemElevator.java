package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.RobotConstants.ELEVATOR_MULTIPLIER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class SubSystemElevator {
    // Instantiate the drivetrain motor variables
    private DcMotorEx elevator;
    private double multiplier;


    public SubSystemElevator(HardwareMap hardwareMap, double multiplier) throws InterruptedException {                 // Motor Mapping
        // Initialize the motor hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        elevator = hardwareMap.get(DcMotorEx.class, "elevator");       //Sets the names of the hardware on the hardware map
        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setPower(1);
        this.multiplier = multiplier;
    }


    public void setPosition(int encoderPosition) {
        elevator.setTargetPosition((int) (multiplier * encoderPosition));
    }

    public int getPosition() {
        return (int)(RobotConstants.ELEVATOR_MULTIPLIER * elevator.getCurrentPosition());
    }
}