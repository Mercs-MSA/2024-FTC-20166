package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;

public class SubSystemIntakeSlide
{
    private Servo intakeSlideServo;

    private double targetServoPosition = 0;

    private double currentServoPosition = 0;

    private double sliderSpeed = 0.01;


    public SubSystemIntakeSlide(HardwareMap hardwareMap) throws InterruptedException {

        intakeSlideServo = hardwareMap.get(Servo.class, "intakeSlideServo");
    }

    public double min = 0.25;
    public double max = 0.8;

    public void setPosition(double position)
    {
        if (position < 0)
        {
            position = 0;
        }
        else if (position > 1)
        {
            position = 1;
        }
        targetServoPosition = min+((max-min) * position);
    }
    public void updateServoPosition()
    {
        if (targetServoPosition > currentServoPosition)
        {
            currentServoPosition = currentServoPosition + sliderSpeed;
            if (currentServoPosition > targetServoPosition)
            {
                currentServoPosition = targetServoPosition;
            }
        }
        else if (targetServoPosition < currentServoPosition)
        {
            currentServoPosition = currentServoPosition - sliderSpeed;
            if (currentServoPosition < targetServoPosition)
            {
                currentServoPosition = targetServoPosition;
            }
        }

        intakeSlideServo.setPosition(currentServoPosition);

    }

}
