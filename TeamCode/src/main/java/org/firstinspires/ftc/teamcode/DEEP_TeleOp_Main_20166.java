package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemArm;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemElevator;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemGrabber;

@TeleOp
public class DEEP_TeleOp_Main_20166 extends LinearOpMode {

    private static final int ELEVATOR_BOTTOM_POSITION = 0;
    private static final int ELEVATOR_TOP_RUNG_PLACE = -811;

    private static final int ELEVATOR_TOP_RUNG_RELEASE = -574;

    private static final int ELEVATOR_BOTTOM_RUNG_PLACE = -400;

    private static final int ELEVATOR_BOTTOM_RUNG_RELEASE = -225;

    private static final int ELEVATOR_TOP_BASKET = -1401;

    private static final int ELEVATOR_MIDDLE_BASKET = -896;

    private static final int ELEVATOR_SPECIMEN_PICKUP = -263;
    private static final int ELEVATOR_TEST_CHANGE = 50;
    private static final double GRABBER_OPEN_POSITION = 0.7;
    private static final double GRABBER_CLOSE_POSITION = 0.95;
    private int elevatorMoveTo = 0;
    private double translateX;
    private double translateY;
    private double joy1RightX;
    private double joy1RightY;//
    private double FLMP, FRMP, BLMP, BRMP;

    private boolean elevatorMoveBottom = false;
    private boolean elevatorMoveTop = false;
    private boolean elevatorMoveLow = false;
    private boolean elevatorMoveHigh = false;

    private boolean grabberOpen;
    private boolean grabberClose;

    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    private IMU imu         = null;

    private SubSystemElevator robotElevator = null;
    private SubSystemGrabber robotGrabber = null;

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

    private void initializeSubSystems() throws InterruptedException {
        robotElevator = new SubSystemElevator(hardwareMap);
        robotGrabber = new SubSystemGrabber(hardwareMap);
        robotGrabber.setPosition(GRABBER_OPEN_POSITION);
    }

    private void initalizeEverything() throws InterruptedException {
        initializeDriveMotors();
        initializeSensors();
        initializeSubSystems();
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
        //driving
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

        elevatorMoveBottom = gamepad2.dpad_down;
        elevatorMoveTop = gamepad2.dpad_up;
        elevatorMoveLow = gamepad2.dpad_left;
        elevatorMoveHigh = gamepad2.dpad_right;

        grabberOpen = gamepad2.x;
        grabberClose = gamepad2.b;

        //buttons

        //if(gamepad1.left_bumper) arm.setPosition(-786); else arm.setPosition(0);
    }

    private void updateDashboard()
    {
        telemetry.addData("FL",frontLeftDrive.getVelocity());
        telemetry.addData("FR",frontRightDrive.getVelocity());
        telemetry.addData("BL",backLeftDrive.getVelocity());
        telemetry.addData("BR",backRightDrive.getVelocity());
        telemetry.addData("Heading", getHeading());

        telemetry.addLine("\n");
        telemetry.addData("Elevator Pos", robotElevator.getPosition());
        telemetry.addData("Elevator moveto", elevatorMoveTo);
        telemetry.addData("left bumper", gamepad1.left_bumper);
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
    public void updateElevator()
    {
        if (elevatorMoveBottom == true)
        {
            robotElevator.setPosition(ELEVATOR_BOTTOM_POSITION);
            elevatorMoveTo = ELEVATOR_BOTTOM_POSITION;
        }
        else if (elevatorMoveTop == true)
        {
            robotElevator.setPosition(ELEVATOR_TOP_RUNG_PLACE);
            elevatorMoveTo = ELEVATOR_TOP_RUNG_PLACE;
        }
        else if (elevatorMoveLow == true)
        {
            robotElevator.setPosition(ELEVATOR_SPECIMEN_PICKUP);
            elevatorMoveTo = ELEVATOR_TOP_BASKET;
        }
        else if (elevatorMoveHigh == true)
        {
            robotElevator.setPosition(ELEVATOR_TOP_RUNG_RELEASE);
            elevatorMoveTo = ELEVATOR_TOP_RUNG_RELEASE;
        }
        else if (gamepad2.right_bumper == true)
        {
            robotElevator.setPosition(robotElevator.getPosition() + ELEVATOR_TEST_CHANGE);
        }
        else if (gamepad2.left_bumper == true)
        {
            robotElevator.setPosition(robotElevator.getPosition() - ELEVATOR_TEST_CHANGE);
        }
    }
    public void updateGrabber()
    {
       if (grabberOpen == true)
       {
           robotGrabber.setPosition(GRABBER_OPEN_POSITION);
       }
       else if (grabberClose == true)
        {
            robotGrabber.setPosition(GRABBER_CLOSE_POSITION);
        }
    }
        public void runOpMode() throws InterruptedException {
        initalizeEverything();

        waitForStart();

        while (opModeIsActive())
        {
            updateJoysticks();
            calculateDrivebaseSpeed();
            updateDrivebaseMotors(FLMP, FRMP, BLMP, BRMP);
            updateElevator();
            updateGrabber();
            updateDashboard();
        }

    }
}




