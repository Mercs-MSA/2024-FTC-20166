package org.firstinspires.ftc.teamcode.Subsystems;
//elevator
//

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class SubSystemGrabber {
    // Instantiate the drivetrain motor variables
    private Servo grabberLeft;
    private Servo grabberRight;
    private RobotConstants robotConstants;


    public SubSystemGrabber(HardwareMap hardwareMap, boolean open) throws InterruptedException
    {
         grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
         grabberRight = hardwareMap.get(Servo.class, "grabberRight");
         setPosition(open ? robotConstants.GRABBER_OPEN_POSITION : robotConstants.GRABBER_CLOSE_POSITION);
     }

    public void setPosition(double position) {
        grabberLeft.setPosition(position);
        grabberRight.setPosition(1.0 - position);

    }

    public double getLeftPosition() {
        return grabberLeft.getPosition();
    }
    public double getRightPosition() {
        return grabberRight.getPosition();
    }

}