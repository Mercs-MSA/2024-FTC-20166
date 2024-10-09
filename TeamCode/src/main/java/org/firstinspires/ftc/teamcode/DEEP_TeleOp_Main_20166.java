package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemElevator;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemGrabber;

@TeleOp
public class DEEP_TeleOp_Main_20166 extends LinearOpMode {

    private static final int ELEVATOR_BOTTOM_POSITION = 0;//Lowest position, on the floor
    private static final int ELEVATOR_TOP_RUNG_PLACE = -1990;//Upper rung starting position

    private static final int ELEVATOR_TOP_RUNG_RELEASE = -1450;//Upper rung pull down position

 //   private static final int ELEVATOR_BOTTOM_RUNG_PLACE = -400;

 //   private static final int ELEVATOR_BOTTOM_RUNG_RELEASE = -225;

 //   private static final int ELEVATOR_TOP_BASKET = -1401;

 //   private static final int ELEVATOR_MIDDLE_BASKET = -896;

    private static final int ELEVATOR_SPECIMEN_PICKUP = -658;
    private static final int ELEVATOR_TEST_CHANGE = 50;
    private static final double GRABBER_OPEN_POSITION = 0.7;
    private static final double GRABBER_CLOSE_POSITION = 0.95;
    private static final int OBSERVATION_PICKUP_LEFT_DISTANCE = 500;
    private static final int OBSERVATION_PICKUP_FRONT_DISTANCE = 45;

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

    NormalizedColorSensor colorSensor;
    RevBlinkinLedDriver blinkinLedDriver;
    Rev2mDistanceSensor leftDistanceSensor;
    Rev2mDistanceSensor frontDistanceSensor;

    private boolean driverAssistPickup = false;

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
        //Initialize gyro etc...
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        //Initialize the color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        //Initialize distance sensors
        leftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistanceSensor");
        frontDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "frontDistanceSensor");

    }

    private void initializeSubSystems() throws InterruptedException {
        robotElevator = new SubSystemElevator(hardwareMap);
        robotGrabber = new SubSystemGrabber(hardwareMap);
        robotGrabber.setPosition(GRABBER_OPEN_POSITION);
    }

    private void initializeLEDs(){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    private void initalizeEverything() throws InterruptedException {
        initializeDriveMotors();
        initializeSensors();
        initializeSubSystems();
        initializeLEDs();
    }

    public String getSampleColor()
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        float maxsat = Math.max(Math.max(colors.red, colors.green), colors.blue);
        float r = colors.red/maxsat;
        float g = colors.green/maxsat;
        float b = colors.blue/maxsat;

        String sample = "Nothing";

        if (distance < 3.0) {
            if (r == 1.0) {
                sample = "Red";
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (b == 1.00) {
                sample = "Blue";
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else {
                sample = "Yellow";
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }
        }
        else {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
        return sample;
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }

    public double getHeadingDegrees() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
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

        if (gamepad1.right_bumper){//Switch to robot centric
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

        if (gamepad1.x)
            driverAssistPickup = true;
        else
            driverAssistPickup = false;

        //buttons

        //if(gamepad1.left_bumper) arm.setPosition(-786); else arm.setPosition(0);
    }

    private void updateDashboard()
    {
//        telemetry.addData("FL",frontLeftDrive.getCurrentPosition());
//        telemetry.addData("FR",frontRightDrive.getCurrentPosition());
//        telemetry.addData("BL",backLeftDrive.getCurrentPosition());
//        telemetry.addData("BR",backRightDrive.getCurrentPosition());
        telemetry.addData("Heading", getHeadingDegrees());

        telemetry.addLine("\n");
        telemetry.addData("Elevator Pos", robotElevator.getPosition());
        telemetry.addData("Elevator moveto", elevatorMoveTo);

        telemetry.addLine("\n");
        telemetry.addData("Sample detected", getSampleColor());

        telemetry.addData("Left distance", leftDistanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Front distance", frontDistanceSensor.getDistance(DistanceUnit.MM));
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
    private void setElevator(int position)
    {
        robotElevator.setPosition(position);
        elevatorMoveTo = position;
    }
    public void updateElevator()
    {
        if (elevatorMoveBottom == true)
        {
            setElevator(ELEVATOR_BOTTOM_POSITION);
        }
        else if (elevatorMoveTop == true)
        {
            setElevator(ELEVATOR_TOP_RUNG_PLACE);
        }
        else if (elevatorMoveLow == true)
        {
            setElevator(ELEVATOR_SPECIMEN_PICKUP);
        }
        else if (elevatorMoveHigh == true)
        {
            setElevator(ELEVATOR_TOP_RUNG_RELEASE);
        }
        else if (gamepad2.right_bumper == true)
        {
            setElevator(robotElevator.getPosition() + ELEVATOR_TEST_CHANGE);
        }
        else if (gamepad2.left_bumper == true)
        {
            setElevator(robotElevator.getPosition() - ELEVATOR_TEST_CHANGE);
        }
    }
    private void setGrabberServo (boolean state)
    {
        if (state)
            robotGrabber.setPosition(GRABBER_OPEN_POSITION);
        else
            robotGrabber.setPosition(GRABBER_CLOSE_POSITION);
    }
    public void updateGrabber()
    {
       if (grabberOpen == true)
       {
           setGrabberServo(true);
       }
       else if (grabberClose == true)
        {
            setGrabberServo(false);
        }
    }

    private boolean rotateRobotToHeading(double heading)
    {
        double rotateSpeed = 0.45;
        double tolerance = 3;
        double headingError = getHeadingDegrees() - heading;
        if (headingError < -180)
            headingError = headingError + 360;
        else if (headingError > 180)
            headingError = headingError - 360;
        if (headingError < -tolerance)
        {
            updateDrivebaseMotors(rotateSpeed, -rotateSpeed, rotateSpeed,-rotateSpeed);
            return false;
        }
        else if (headingError > tolerance)
        {
            updateDrivebaseMotors(-rotateSpeed, rotateSpeed, -rotateSpeed, rotateSpeed);
            return false;
        }
        else
        {
            updateDrivebaseMotors(0, 0, 0, 0);
            return true;
        }
    }
    private boolean driveRobotToDistanceFrom (double distance)
    {
        double currentDistance = frontDistanceSensor.getDistance(DistanceUnit.MM);
        double driveSpeed = -0.25;
        if (currentDistance > distance)
        {
            updateDrivebaseMotors(driveSpeed, driveSpeed, driveSpeed, driveSpeed);
            return false;

        }
            else
        {
            updateDrivebaseMotors(0, 0, 0, 0);
            return true;
        }
    }
    private static final int STATE_WAIT = 0;
    private static final int STATE_FACE_WALL = 1;
    private static final int STATE_DRIVE_TO_WALL_1 = 2;
    private static final int STATE_WAIT_RELEASE = 3;
    private static final int STATE_STRAFE_TO_WALL = 4;

    private int currentDriverAssistState = STATE_WAIT;

    private void processStateMachine()
    {
        boolean done = false;
        if (currentDriverAssistState == STATE_WAIT)
        {
            if (driverAssistPickup)
            {
                currentDriverAssistState = STATE_FACE_WALL;
            }
        }
        else if (currentDriverAssistState == STATE_FACE_WALL)
        {
            done = rotateRobotToHeading(180.0);
            if (done)
                currentDriverAssistState = STATE_DRIVE_TO_WALL_1;
        }
        else if (currentDriverAssistState == STATE_DRIVE_TO_WALL_1)
        {
            done = driveRobotToDistanceFrom(120.0);
            if (done)
            {
                currentDriverAssistState = STATE_STRAFE_TO_WALL;
                setGrabberServo(true);
                setElevator(ELEVATOR_SPECIMEN_PICKUP);
            }
        }
        else if (currentDriverAssistState == STATE_WAIT_RELEASE)
        {
            if (!driverAssistPickup)
                currentDriverAssistState = STATE_WAIT;
        }
        else if (currentDriverAssistState == STATE_STRAFE_TO_WALL)
        {
            done = true;
            if (done)
                currentDriverAssistState = STATE_WAIT_RELEASE;
        }
    }

        public void runOpMode() throws InterruptedException {
        initalizeEverything();

        waitForStart();

        while (opModeIsActive())
        {
            updateJoysticks();
            calculateDrivebaseSpeed();
            if (currentDriverAssistState == STATE_WAIT)
            {
                updateDrivebaseMotors(FLMP, FRMP, BLMP, BRMP);
                updateElevator();
                updateGrabber();
            }
            processStateMachine();
            updateDashboard();
        }

    }
}




