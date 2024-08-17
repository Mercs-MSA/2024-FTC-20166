package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class MechanumSimple extends LinearOpMode {

    private double joy1LeftX;
    private double joy1LeftY;
    private double joy1RightX;
    private double joy1RightY;//
    private double FLMP, FRMP, BLMP, BRMP;
    public int maxTicksPer = 1000;


    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

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
        // Get the PIDF coefficients for the RUN_USING_ENCODER RunMode.
        PIDFCoefficients pidfOrig = frontLeftDrive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("P",pidfOrig.p);
        telemetry.addData("I",pidfOrig.i);
        telemetry.addData("D",pidfOrig.d);
        telemetry.addData("F",pidfOrig.f);//
        updateTelemetry(telemetry);
    }

    private static double[] rotatePoint(double xPoint, double yPoint, double angle) {
        double[]  Result = new double[2];

        Result[0] = xPoint * Math.cos(angle) - yPoint * Math.sin(angle);
        Result[1] = xPoint * Math.sin(angle) + yPoint * Math.cos(angle);
        return Result;
    }

    private void updateJoysticks()
    {
        joy1LeftX = gamepad1.left_stick_x;
        joy1LeftY = gamepad1.left_stick_y;
        joy1RightX = gamepad1.right_stick_x;
        joy1RightY = gamepad1.right_stick_y;
    }

    private void updateDashboard()
    {
        telemetry.addData("FL",frontLeftDrive.getVelocity());
        telemetry.addData("FR",frontRightDrive.getVelocity());
        telemetry.addData("BL",backLeftDrive.getVelocity());
        telemetry.addData("BR",backRightDrive.getVelocity());
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
/*
        frontLeftDrive.setPower(FLMP);
        frontRightDrive.setPower(FRMP);
        backLeftDrive.setPower(BLMP);
        backRightDrive.setPower(BRMP);
*/
        frontLeftDrive.setVelocity(FLMP * maxTicksPer);
        frontRightDrive.setVelocity(FRMP * maxTicksPer);
        backLeftDrive.setVelocity(BLMP * maxTicksPer);
        backRightDrive.setVelocity(BRMP * maxTicksPer);
    }
    private void calculateDrivebaseSpeed()
    {
        double FLPFB, FRPFB, BLPFB, BRPFB;
        double FLPLR, FRPLR, BLPLR, BRPLR;
        double FLPR, FRPR, BLPR, BRPR;

        FLPFB = joy1LeftY;
        FRPFB = joy1LeftY;
        BLPFB = joy1LeftY;
        BRPFB = joy1LeftY;

        FLPLR = joy1LeftX;
        FRPLR = -joy1LeftX;
        BLPLR = -joy1LeftX;
        BRPLR = joy1LeftX;

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




