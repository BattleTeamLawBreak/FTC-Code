/*
Copyright 2025 

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous

public class AutonRight extends LinearOpMode {
    private DcMotor frontleftDrive = null;
    private DcMotor backleftDrive = null;
    private DcMotor frontrightDrive = null;
    private DcMotor backrightDrive = null;
    
    Servo wrist = null;
    Servo claw = null;
    private DcMotor leftArm;  
    private DcMotor rightArm;
    
    IMU imu;
    
    double kp = 0.8;
    double ki = 0.5;
    double kd = 0.1;
    double integralSum = 0;
    double previousPropError = 0;
    private ElapsedTime runtime = new ElapsedTime();
    
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 12;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) /  WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    @Override
    public void runOpMode() {
        //frontleftDrive = hardwareMap.get(DcMotor.class, "rearleft");
        //frontrightDrive = hardwareMap.get(DcMotor.class, "rearright");
        //backleftDrive = hardwareMap.get(DcMotor.class, "frontright");
        //backrightDrive = hardwareMap.get(DcMotor.class, "frontleft");

        frontrightDrive = hardwareMap.get(DcMotor.class, "frontright");
        frontleftDrive = hardwareMap.get(DcMotor.class, "frontleft");
        backrightDrive = hardwareMap.get(DcMotor.class, "rearright");
        backleftDrive = hardwareMap.get(DcMotor.class, "rearleft");
        
        wrist = hardwareMap.get(Servo.class, "servo2");
        claw = hardwareMap.get(Servo.class, "servo3");
        imu = hardwareMap.get(IMU.class, "imu");
 
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
       
        frontleftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontrightDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.FORWARD);
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftArm = hardwareMap.get(DcMotor.class, "motor3");
        rightArm = hardwareMap.get(DcMotor.class, "motor4");

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();
        imu.resetYaw();

        drive(60, 45, 0.2);
    }

    private void drive(double distance, double heading, double power) {
        int target = (int)(distance * DRIVE_COUNTS_PER_IN);

        frontleftDrive.setTargetPosition(target + frontleftDrive.getCurrentPosition());
        frontrightDrive.setTargetPosition(target + frontrightDrive.getCurrentPosition());
        backleftDrive.setTargetPosition(target + backleftDrive.getCurrentPosition());
        backrightDrive.setTargetPosition(target + backrightDrive.getCurrentPosition());
    
        frontleftDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontrightDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleftDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backrightDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        
        frontleftDrive.setPower(power);
        frontrightDrive.setPower(power);
        backleftDrive.setPower(power);
        backrightDrive.setPower(power);
        
        while (opModeIsActive() || frontleftDrive.isBusy() || frontrightDrive.isBusy() || backleftDrive.isBusy() || backrightDrive.isBusy()){
            
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double headingCorrection = PIDcontroller(heading, orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Current Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Target Heading ", heading);
            telemetry.addData("Position: ", frontleftDrive.getCurrentPosition());
            telemetry.addData("Target ", target);
            telemetry.addData("Inches ", distance);

            // Normalize speeds if either one exceeds +/- 1.0;
            double max = Math.max(Math.abs(power), Math.abs(headingCorrection));
            if (max > 1.0)
            {
                power /= max;
                headingCorrection /= max;
            }
        
            frontleftDrive.setPower((power + headingCorrection) * 0.3);
            frontrightDrive.setPower((power - headingCorrection) * 0.3);
            backleftDrive.setPower((power + headingCorrection) * 0.3);
            backrightDrive.setPower((power - headingCorrection) * 0.3);
            
            telemetry.addData("power: ", frontleftDrive.getPower());
            telemetry.addData("power: ", frontrightDrive.getPower()); 
            telemetry.addData("power: ", backleftDrive.getPower());
            telemetry.addData("power: ", backrightDrive.getPower());
            telemetry.update();
            
        }

    }
    
    public double PIDcontroller(double targetHeading, double currentHeading) {

        double propError = angleWrap(targetHeading - currentHeading);

        integralSum += propError * runtime.seconds();

        double derivativeError = (propError - previousPropError)/runtime.time();
        runtime.reset();

        previousPropError = propError;

        propError *= kp;
        integralSum *= ki;
        derivativeError *= kd;

        double output = propError + integralSum + derivativeError;
        
        telemetry.addData("error: ", output);

        return output;
    }
    

    public double angleWrap(double degrees) {

        while (degrees > 360) {
            degrees -= 360;
        }
        while (degrees <= -360) {
            degrees += 360;
        }
    
        return degrees;
    }

}
