package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
@Disabled
public class MechanumSimple extends LinearOpMode {

    private double translateX;
    private double translateY;
    private double joy1RightX;
    private double joy1RightY;//
    private double FLMP, FRMP, BLMP, BRMP;


    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    private IMU imu         = null;

    public void initializeDriveMotors()
    {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BR");

        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initializeSensors()
    {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }

    private static double[] rotatePoint(double xPoint, double yPoint, double angle) {
        double[]  Result = new double[2];

        Result[0] = xPoint * Math.cos(angle) - yPoint * Math.sin(angle);
        Result[1] = xPoint * Math.sin(angle) + yPoint * Math.cos(angle);
        return Result;
    }

    private static double rotatePointX(double xPoint, double yPoint, double angle) {
        double[]  Result = new double[2];

        return xPoint * Math.cos(angle) - yPoint * Math.sin(angle);
    }

    private static double rotatePointY(double xPoint, double yPoint, double angle) {
        double[]  Result = new double[2];

        return xPoint * Math.sin(angle) + yPoint * Math.cos(angle);
    }

    private void updateJoysticks()
    {
        double x, y, heading;

        heading = getHeading();

        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        joy1RightX = gamepad1.right_stick_x;
        joy1RightY = gamepad1.right_stick_y;





        if (gamepad1.right_bumper){
            translateX = x;
            translateY = y;
        }
        else{
            translateX = rotatePointX(x, y, heading);
            translateY = rotatePointY(x, y, heading);
        }

    }

    private void updateDashboard()
    {
        telemetry.addData("FL",frontLeftDrive.getVelocity());
        telemetry.addData("FR",frontRightDrive.getVelocity());
        telemetry.addData("BL",backLeftDrive.getVelocity());
        telemetry.addData("BR",backRightDrive.getVelocity());
        telemetry.addData("Heading", getHeading());
        updateTelemetry(telemetry);
    }
    private void updateDrivebaseMotors(double FLMP, double FRMP, double BLMP, double BRMP)
    {
        double maxPower;

        maxPower = Math.max(Math.max(FLMP, FRMP), Math.max(BLMP, BRMP));

        if (maxPower > 1.0)
        {
            FLMP = FLMP / maxPower;
            FRMP = FRMP / maxPower;
            BLMP = BLMP / maxPower;
            BRMP = BRMP / maxPower;
        }
        frontLeftDrive.setPower(FLMP);
        frontRightDrive.setPower(FRMP);
        backLeftDrive.setPower(BLMP);
        backRightDrive.setPower(BRMP);
    }
    private void calculateDrivebaseSpeed()
    {
        double FLPFB, FRPFB, BLPFB, BRPFB;
        double FLPLR, FRPLR, BLPLR, BRPLR;
        double FLPR, FRPR, BLPR, BRPR;

        FLPFB = translateY;
        FRPFB = translateY;
        BLPFB = translateY;
        BRPFB = translateY;

        FLPLR = -translateX;
        FRPLR = translateX;
        BLPLR = translateX;
        BRPLR = -translateX;

        FLPR = -joy1RightX;
        FRPR = joy1RightX;
        BLPR = -joy1RightX;
        BRPR = joy1RightX;

        FLMP = FLPFB + FLPLR + FLPR;
        FRMP = FRPFB + FRPLR + FRPR;
        BLMP = BLPFB + BLPLR + BLPR;
        BRMP = BRPFB + BRPLR + BRPR;
    }
    public void runOpMode()  {
        initializeSensors();
        initializeDriveMotors();

        waitForStart();
        while (opModeIsActive())
        {
            updateJoysticks();
            calculateDrivebaseSpeed();
            updateDrivebaseMotors(FLMP, FRMP, BLMP, BRMP);
            updateDashboard();
        }
    }
}




