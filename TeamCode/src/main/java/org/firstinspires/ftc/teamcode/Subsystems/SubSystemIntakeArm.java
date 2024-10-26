package org.firstinspires.ftc.teamcode.Subsystems;
//elevator
//

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class SubSystemIntakeArm {
    // Instantiate the drivetrain motor variables
    private Servo leftServo;
    private Servo rightServo;
    private RobotConstants robotConstants = new RobotConstants();


    public SubSystemIntakeArm(HardwareMap hardwareMap) throws InterruptedException
    {
         leftServo = hardwareMap.get(Servo.class, "leftIntakeArmServo");
         rightServo = hardwareMap.get(Servo.class, "rightIntakeArmServo");
         setPosition(robotConstants.INTAKE_ARM_UP_POSITION);
    }

    public void setPosition(double position)
    {
        leftServo.setPosition(1 - position);
        rightServo.setPosition(position);
    }

}