package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.SubSystemGrabber;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakeSlide;

@TeleOp
//@Disabled
public class MotorTest extends LinearOpMode {

    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    private SubSystemGrabber robotGrabber = null;
    private SubSystemIntakeSlide robotIntakeSlide = null;

    private static final double GRABBER_OPEN_POSITION = 0.7;
    private static final double GRABBER_CLOSE_POSITION = 0.95;
    public void initializeDriveMotors()
    {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BR");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
private void initalizeGrabber() throws InterruptedException
{
    robotGrabber = new SubSystemGrabber(hardwareMap);
    robotGrabber.setPosition(GRABBER_OPEN_POSITION);
}
    private void setGrabberServo (boolean state)
    {
        if (state)
            robotGrabber.setPosition(GRABBER_OPEN_POSITION);
        else
            robotGrabber.setPosition(GRABBER_CLOSE_POSITION);
    }

    private void inttializeIntakeSlide() throws InterruptedException
    {
        robotIntakeSlide = new SubSystemIntakeSlide(hardwareMap);

    }


    public void runOpMode() throws InterruptedException {
        initializeDriveMotors();
        initalizeGrabber();
        inttializeIntakeSlide();

        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("FL (0)", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR (1)", frontRightDrive.getCurrentPosition());
            telemetry.addData("BL (2)", backLeftDrive.getCurrentPosition());
            telemetry.addData("BR (3)", backRightDrive.getCurrentPosition());
            telemetry.addData("Intake Slide", robotIntakeSlide.getPosition());
            updateTelemetry(telemetry);

            if(gamepad1.x)
                frontLeftDrive.setPower(-gamepad1.left_stick_y);
            else
                frontLeftDrive.setPower(0);

            if(gamepad1.y)
                frontRightDrive.setPower(-gamepad1.left_stick_y);
            else
                frontRightDrive.setPower(0);

            if(gamepad1.a)
                backLeftDrive.setPower(-gamepad1.left_stick_y);
            else
                backLeftDrive.setPower(0);

            if(gamepad1.b)
                backRightDrive.setPower(-gamepad1.left_stick_y);
            else
                backRightDrive.setPower(0);
            if (gamepad1.dpad_left)
                setGrabberServo(true);
            if (gamepad1.dpad_right)
                setGrabberServo(false);

            robotIntakeSlide.movePosition(gamepad1.left_trigger, gamepad1.right_trigger);

        }
    }
}




