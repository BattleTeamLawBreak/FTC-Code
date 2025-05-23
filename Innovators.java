package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Innovators", group="Linear OpMode")

public class Innovators extends LinearOpMode {

    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;
    DcMotorEx arm;
    IMU imu;
    static double headingOffset = 0;
    double integralSum = 0;
    static double kp = 0.02;
    static double ki = 0;
    static double kd = 0.001;
    static double f = 0.8;
    double target = 0;
    double lastError = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    final double ticks_in_degrees = 537.7 / 360;
    final double minArmPos = 0;
    final double maxArmPos = 380;

    @Override
    public void runOpMode() {
        init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double jx = gamepad1.left_stick_y;
            double jy = gamepad1.left_stick_x;
            double jw = gamepad1.right_stick_x;
            double lTrigger = gamepad1.left_trigger;
            double rTrigger = gamepad1.right_trigger;

            driveFieldXYW(jx, jy, jw);

            moveArm(rTrigger, lTrigger);

            telemetry.addData("Status", "Running");
            telemetry.addData("IMU heading", getIMUHeading());
            telemetry.update();
        }
    }

    private void moveArm(double rTrigger, double lTrigger) {
        int armPos = arm.getCurrentPosition();
        if (rTrigger > 0) {
            target += rTrigger;
        }
        if (lTrigger > 0){
            target -= lTrigger;
        }

        if (target > maxArmPos){
            target = maxArmPos;
        }
        if (target < minArmPos){
            target = minArmPos;
        }
        arm.setPower(pidController(target, armPos));
    }

    public void init(HardwareMap hardwareMap) {
        lf = initDcMotor(hardwareMap, "fl", DcMotor.Direction.REVERSE);
        rf = initDcMotor(hardwareMap, "fr", DcMotor.Direction.FORWARD);
        lb = initDcMotor(hardwareMap, "bl", DcMotor.Direction.REVERSE);
        rb = initDcMotor(hardwareMap, "br", DcMotor.Direction.FORWARD);
        arm = initDcMotor(hardwareMap, "arm", DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initIMU(hardwareMap);
    }

    public DcMotorEx initDcMotor(HardwareMap hardwareMap,
                                 String name,
                                 DcMotor.Direction dir) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }

    public void driveXYW(double rx, double ry, double rw) {
        double lfPower = rx - ry - rw;
        double rfPower = rx + ry + rw;
        double lbPower = rx + ry - rw;
        double rbPower = rx - ry + rw;

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
    }

    public void driveFieldXYW(double fx, double fy, double fw) {
        // rotate field orientation to robot orientation
        double theta = Math.toRadians(getHeading());
        double rx = fx * Math.cos(-theta) - fy * Math.sin(-theta);
        double ry = fx * Math.sin(-theta) + fy * Math.cos(-theta);

        driveXYW(rx, ry, fw);
    }

    public double pidController(double target, double state){
        double error = target - state;
        integralSum += error * elapsedTime.seconds();
        double derivative = (error - lastError) / elapsedTime.seconds();
        lastError = error;
        elapsedTime.reset();
        double pid = (error * kp) + (derivative * kd) + (integralSum * ki);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        telemetry.addData("pid: ", pid);
        telemetry.addData("ff: ", ff);
        telemetry.addData("target: ", target);
        telemetry.addData("armPos", state);

        return pid + ff;
    }

    public void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(params);
    }

    public double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setHeading(double h) {
        headingOffset = h - getIMUHeading();
    }

    public double getHeading() {
        return AngleUnit.normalizeDegrees(headingOffset + getIMUHeading());
    }

}
