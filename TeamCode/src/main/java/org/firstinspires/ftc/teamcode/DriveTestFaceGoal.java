package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp
//@Disabled
public class DriveTestFaceGoal extends LinearOpMode {

    private double driveX;
    private double driveY;
    SparkFunOTOS myOtos;

    //Motor demo variables
    private DcMotorEx m0 = null;
    private DcMotorEx m1 = null;
    private DcMotorEx m2 = null;
    private DcMotorEx m3 = null;
//    private DcMotorEx m4 = null;
//    private DcMotorEx m5 = null;
//    private DcMotorEx m6 = null;
//    private DcMotorEx m7 = null;
    private IMU imu;

    private double FLYPower = 0.0;
    private double FRYPower = 0.0;
    private double BLYPower = 0.0;
    private double BRYPower = 0.0;
    private double FLXPower = 0.0;
    private double FRXPower = 0.0;
    private double BLXPower = 0.0;
    private double BRXPower = 0.0;

    private double FLRPower = 0.0;
    private double FRRPower = 0.0;
    private double BLRPower = 0.0;
    private double BRRPower = 0.0;

    private double driveRotate;


    SparkFunOTOS.Pose2D robotPos;


    public static SparkFunOTOS.Pose2D startPointMiddleBottom = new SparkFunOTOS.Pose2D(0, 60, Math.toRadians(0));

    private double redGoalPointx = 72;
    private double redGoalPointy = 72;

    private double blueGoalPointx = -72;
    private double blueGoalPointy = 72;

    private boolean currentlyTurning = false;
    private double joystickHeading;

    //public Point startPose

    public void initializeHardware()
    {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setPosition(startPointMiddleBottom);
       // myOtos.
        m0 = hardwareMap.get(DcMotorEx.class, "FL");
        m1 = hardwareMap.get(DcMotorEx.class, "FR");
        m2 = hardwareMap.get(DcMotorEx.class, "BL");
        m3 = hardwareMap.get(DcMotorEx.class, "BR");
//        m4 = hardwareMap.get(DcMotorEx.class, "M4");
//        m5 = hardwareMap.get(DcMotorEx.class, "M5");
//        m6 = hardwareMap.get(DcMotorEx.class, "M6");
//        m7 = hardwareMap.get(DcMotorEx.class, "M7");

        m0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        m7.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();
    }

    public double getHeadingDegrees() {
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // return orientation.getYaw(AngleUnit.DEGREES);
        robotPos = myOtos.getPosition();
        return robotPos.h;
    }
    private void setDriveMotors(double FL, double FR, double BL, double BR)
    {
        double greatest = Math.max(Math.max(FL, FR), Math.max(BL, BR));
        if (greatest > 1.0)
        {
            FL = FL/greatest;
            FR = FR/greatest;
            BL = BL/greatest;
            BR = BR/greatest;
        }
        m0.setPower(FL);
        m1.setPower(FR);
        m2.setPower(BL);
        m3.setPower(BR);

    }

    public static double getPointsHeading(double x, double y, double xr, double yr)
    {
        double calculatedAngleRads = Math.atan2(y - yr, x - xr);
        double calculatedAngleDegs = Math.toDegrees(calculatedAngleRads);
        //double correctedAngle = calculatedAngleDegs - 90.0;
        return calculatedAngleDegs;
    }

    public static double headingError(double actualHeading, double desiredHeading)
    {
        double error = actualHeading - desiredHeading;
        if (error > 180)
        {
            error -= 360;
        } else if (error < -180)
        {
            error += 360;
        }
        return error;
    }

    private void updateDriveControls() 
    {
        double angleInRadians;
        double oldDriveX = gamepad1.left_stick_x;
        double oldDriveY = gamepad1.left_stick_y;
        double headingPFactor = (1.0 / 90.0);
        double desiredHeading;
        double goalHeading;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        if(gamepad1.right_bumper)
        {
            angleInRadians = 0;
        }
        else
        {
            angleInRadians = orientation.getYaw(AngleUnit.RADIANS);
        }

        driveX = oldDriveX * Math.cos(angleInRadians) - oldDriveY * Math.sin(angleInRadians);
        driveY = oldDriveX * Math.sin(angleInRadians) + oldDriveY * Math.cos(angleInRadians);
        driveRotate = gamepad1.right_stick_x;
        if (Math.abs(driveRotate) < .1)
        {
            driveRotate = 0;
            if (currentlyTurning)
            {
                joystickHeading = getHeadingDegrees();
            }
            currentlyTurning = false;
        }
        else
        {
            currentlyTurning = true;
        }
        goalHeading = getPointsHeading(redGoalPointx, redGoalPointy, robotPos.x, robotPos.y);
        if (gamepad1.left_bumper)
        {
            desiredHeading = goalHeading;
        } else
        {
            desiredHeading = joystickHeading;
        }
        double error = headingError(getHeadingDegrees(), desiredHeading);
        if (error > 5)
        {
            driveRotate = error * headingPFactor;
        }
        //displaying the error on the driver hub
        telemetry.addData("error", error);
    }

    private void calculateDrivePower()
    {
        FLYPower = -driveY;
        FRYPower = -driveY;
        BLYPower = -driveY;
        BRYPower = -driveY;

        FLXPower = driveX;
        FRXPower = -driveX;
        BLXPower = -driveX;
        BRXPower = driveX;


        FLRPower = driveRotate; //gamepad1.right_stick_x;
        FRRPower = -driveRotate; //-gamepad1.right_stick_x;
        BLRPower = driveRotate; //gamepad1.right_stick_x;
        BRRPower = -driveRotate; //-gamepad1.right_stick_x;
    }

    public void runOpMode() throws InterruptedException {
       initializeHardware();

        waitForStart();
        while (opModeIsActive())
        {
 /*           telemetry.addData("Motor 0", m0.getCurrentPosition());
            telemetry.addData("Motor 1", m1.getCurrentPosition());
            telemetry.addData("Motor 2", m2.getCurrentPosition());
            telemetry.addData("Motor 3", m3.getCurrentPosition());
            telemetry.addData("Motor 4", m4.getCurrentPosition());
            telemetry.addData("Motor 5", m5.getCurrentPosition());
            telemetry.addData("Motor 6", m6.getCurrentPosition());
            telemetry.addData("Motor 7", m7.getCurrentPosition());

  */
//            telemetry.addData("IMU X", orientation.getYaw(AngleUnit.DEGREES));
//            telemetry.addData("IMU Y", orientation.getPitch(AngleUnit.DEGREES));
//            telemetry.addData("IMU Z", orientation.getRoll(AngleUnit.DEGREES));
            updateDriveControls();
            calculateDrivePower();

            //FLXPower.setVelocity(1000);
            //Sets target velocity to 1000 ticks per second
            //m0.setVelocity();
            setDriveMotors((FLXPower + FLYPower + FLRPower), (FRXPower + FRYPower + FRRPower), (BLXPower + BLYPower + BLRPower), (BRXPower + BRYPower + BRRPower));

            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);

            updateTelemetry(telemetry);
        }
    }
}




