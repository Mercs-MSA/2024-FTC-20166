package org.firstinspires.ftc.teamcode.Subsystems;
//elevator
//

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SubSystemIntake {
    // Instantiate the drivetrain motor variables
    private Servo driveServo;


    public SubSystemIntake(HardwareMap hardwareMap) throws InterruptedException {
         driveServo = hardwareMap.get(Servo.class, "driveIntakeServo");
         }

    public void setSpeed(double speed)
    {
        driveServo.setPosition(0.5 + speed/2);
    }

}