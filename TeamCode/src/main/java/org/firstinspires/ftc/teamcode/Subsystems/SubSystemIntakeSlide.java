package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;

public class SubSystemIntakeSlide
{
    private Servo intakeSlideServo;


    public SubSystemIntakeSlide(HardwareMap hardwareMap) throws InterruptedException {

        intakeSlideServo = hardwareMap.get(Servo.class, "intakeSlideServo");
    }

    public double min = 0.25;
    public double max = 0.8;

    public void setPosition(double position)
    {
        double servoPosition;
        if (position < 0)
        {
            position = 0;
        }
        else if (position > 1)
        {
            position = 1;
        }
        servoPosition = min+((max-min) * position);
        intakeSlideServo.setPosition(servoPosition);
    }

}
