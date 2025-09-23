package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "TeleOp Turret Tracker", group = "TeleOp")
public class TeleOpTurretTracker extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor turretMotor = null;
    private Servo turretServo = null; 
    
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private static final double DRIVE_SPEED_MULTIPLIER = 0.8;
    
    private static final double TURRET_KP = 0.025;
    private static final double TURRET_KI = 0.001;
    private static final double TURRET_KD = 0.005;
    private static final double MAX_TURRET_SPEED = 0.6;
    private static final double TURRET_DEADBAND = 1.5;
    private static final double TURRET_SEARCH_SPEED = 0.15;
    
    private static final double TURRET_TICKS_PER_DEGREE = 4.0;
    private double turretTargetPosition = 0;
    private boolean turretTrackingActive = true;
    
    private static final int[] GOAL_TAG_IDS = {1, 2, 3, 4, 7, 8};
    
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime turretSearchTimer = new ElapsedTime();
    
    private PIDController turretPID;
    
    private class PIDController {
        private double kP, kI, kD;
        private double integral, previousError;
        private ElapsedTime timer;
        
        public PIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.integral = 0;
            this.previousError = 0;
            this.timer = new ElapsedTime();
        }
        
        public double calculate(double error) {
            double deltaTime = timer.seconds();
            timer.reset();
            
            if (deltaTime > 0) {
                integral += error * deltaTime;
                double derivative = (error - previousError) / deltaTime;
                
                double output = kP * error + kI * integral + kD * derivative;
                
                previousError = error;
                
                return Range.clip(output, -MAX_TURRET_SPEED, MAX_TURRET_SPEED);
            }
            return 0;
        }
        
        public void reset() {
            integral = 0;
            previousError = 0;
            timer.reset();
        }
        
        public void setIntegralLimit(double limit) {
            integral = Range.clip(integral, -limit, limit);
        }
    }
    
    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();
        initPID();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Left stick: drive, Right stick: turn");
        telemetry.addData("Controls", "Y: Toggle turret tracking, X: Manual turret control");
        telemetry.addData(">", "Touch Play to start TeleOp");
        telemetry.update();
        waitForStart();
        
        runtime.reset();
        turretSearchTimer.reset();
        
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                handleDriveControls();
                handleTurretControls();
                updateTelemetry();
                
                sleep(20);
            }
        }
        
        visionPortal.close();
    }
    
    private void initHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        
        turretMotor = hardwareMap.get(DcMotor.class, "turret_motor");
        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
            .setDrawAxes(false)
            .setDrawCubeProjection(false)
            .setDrawTagOutline(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagProcessor.TagLibrary.getCenterStageTagLibrary())
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .build();
            
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(aprilTag)
            .build();
    }
    
    private void initPID() {
        turretPID = new PIDController(TURRET_KP, TURRET_KI, TURRET_KD);
    }
    
    private void handleDriveControls() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        
        drive *= DRIVE_SPEED_MULTIPLIER;
        strafe *= DRIVE_SPEED_MULTIPLIER;
        turn *= DRIVE_SPEED_MULTIPLIER;
        
        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;
        
        double maxPower = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));
        
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }
        
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    
    private void handleTurretControls() {
        if (gamepad1.y) {
            turretTrackingActive = !turretTrackingActive;
            if (turretTrackingActive) {
                turretPID.reset();
            }
            sleep(200);
        }
        
        if (turretTrackingActive) {
            handleAutomaticTurretTracking();
        } else {
            handleManualTurretControl();
        }
    }
    
    private void handleAutomaticTurretTracking() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection targetTag = findClosestGoalTag(currentDetections);
        
        if (targetTag != null) {
            double bearingError = targetTag.ftcPose.bearing;
            
            if (Math.abs(bearingError) > TURRET_DEADBAND) {
                double turretSpeed = turretPID.calculate(bearingError);
                turretPID.setIntegralLimit(0.3);
                
                if (turretMotor != null) {
                    turretMotor.setPower(-turretSpeed);
                }
                
                turretSearchTimer.reset();
            } else {
                if (turretMotor != null) {
                    turretMotor.setPower(0);
                }
                turretPID.reset();
            }
        } else {
            handleTurretSearch();
            turretPID.reset();
        }
    }
    
    private void handleTurretSearch() {
        if (turretSearchTimer.seconds() < 3.0) {
            if (turretMotor != null) {
                turretMotor.setPower(TURRET_SEARCH_SPEED);
            }
        } else if (turretSearchTimer.seconds() < 6.0) {
            if (turretMotor != null) {
                turretMotor.setPower(-TURRET_SEARCH_SPEED);
            }
        } else {
            turretSearchTimer.reset();
        }
    }
    
    private void handleManualTurretControl() {
        double turretPower = 0;
        
        if (gamepad1.right_bumper) {
            turretPower = 0.3;
        } else if (gamepad1.left_bumper) {
            turretPower = -0.3;
        }
        
        if (Math.abs(gamepad1.right_stick_x) > 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper) {
            turretPower = gamepad1.right_stick_x * 0.4;
        }
        
        if (turretMotor != null) {
            turretMotor.setPower(turretPower);
        }
        
        turretPID.reset();
    }
    
    private AprilTagDetection findClosestGoalTag(List<AprilTagDetection> detections) {
        AprilTagDetection closestTag = null;
        double closestDistance = Double.MAX_VALUE;
        
        for (AprilTagDetection detection : detections) {
            if (isGoalTag(detection.id)) {
                double distance = detection.ftcPose.range;
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestTag = detection;
                }
            }
        }
        
        return closestTag;
    }
    
    private boolean isGoalTag(int tagId) {
        for (int goalId : GOAL_TAG_IDS) {
            if (tagId == goalId) {
                return true;
            }
        }
        return false;
    }
    
    private void updateTelemetry() {
        telemetry.addData("Status", "TeleOp Running: %.1f seconds", runtime.seconds());
        telemetry.addData("Turret Tracking", turretTrackingActive ? "ACTIVE (PID)" : "MANUAL");
        
        if (turretMotor != null) {
            telemetry.addData("Turret Position", turretMotor.getCurrentPosition());
            telemetry.addData("Turret Power", "%.2f", turretMotor.getPower());
        }
        
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection targetTag = findClosestGoalTag(currentDetections);
        
        if (targetTag != null) {
            telemetry.addData("Target Found", "ID %d (%s)", targetTag.id, targetTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", targetTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", targetTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", targetTag.ftcPose.yaw);
        } else {
            telemetry.addData("Target", "No goal tags visible");
        }
        
        telemetry.addData("PID Values", "kP=%.3f, kI=%.3f, kD=%.3f", TURRET_KP, TURRET_KI, TURRET_KD);
        telemetry.addData("Controls", "Y: Toggle tracking, Bumpers: Manual turret");
        telemetry.update();
    }
}
