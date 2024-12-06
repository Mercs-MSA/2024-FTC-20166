package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;

public class SubSystemIntakeSlide {
    /* This code is for the motor driven intake slider
    private DcMotorEx intakeSlide;
    private static final int MAX_EXTENSION_COUNT = 1150;
    private static final int MIN_EXTENSION_COUNT = 0;
     */
    private CRServo intakeSlideServo;


    public SubSystemIntakeSlide(HardwareMap hardwareMap) throws InterruptedException {
        /* This code is for the motor driven intake slider
        intakeSlide = hardwareMap.get(DcMotorEx.class, "intakeSlide");       //Sets the names of the hardware on the hardware map
        intakeSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeSlide.setTargetPosition(0);
        intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlide.setPower(0.7);
         */
        intakeSlideServo = hardwareMap.get(CRServo.class, "intakeSlideServo");
    }


    /* This code is for the motor driven intake slider
    public void setPosition(int encoderPosition) {
        intakeSlide.setTargetPosition(encoderPosition);
    }
    */
    public boolean atMaxPosition()
    {
        return false;
    }
    public boolean atMinPosition()
    {
        return false;
    }
    public void movePosition(double speed)
    {
        if( (speed > 0.0) && (!atMaxPosition()))
            intakeSlideServo.setPower(speed);
        else if ((speed < 0.0) && (!atMinPosition()))
            intakeSlideServo.setPower(speed);
        else  //Should never get here, but good practice to catch all
            intakeSlideServo.setPower(0);
    }
    /* This code is for the motor driven intake slider
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
     */
}
