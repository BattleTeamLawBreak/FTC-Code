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

    // Drive Motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Turret System
    private DcMotor turretMotor = null;
    private Servo turretServo = null; // Alternative: use servo for lighter turrets
    
    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Constants for drive
    private static final double DRIVE_SPEED_MULTIPLIER = 0.8;
    
    // Constants for turret tracking
    private static final double TURRET_SPEED_GAIN = 0.015;  // Turret rotation speed control
    private static final double MAX_TURRET_SPEED = 0.4;     // Maximum turret rotation speed
    private static final double TURRET_DEADBAND = 2.0;      // Degrees - stop turret if within this range
    private static final double TURRET_SEARCH_SPEED = 0.15; // Speed when searching for targets
    
    // Turret position tracking (for motor-based turret)
    private static final double TURRET_TICKS_PER_DEGREE = 4.0; // Adjust based on your turret gearing
    private double turretTargetPosition = 0;
    private boolean turretTrackingActive = true;
    
    // Goal AprilTag IDs (adjust these based on your field setup)
    private static final int[] GOAL_TAG_IDS = {1, 2, 3, 4, 7, 8}; // Example IDs for goal tags
    
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime turretSearchTimer = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        
        // Initialize hardware
        initHardware();
        
        // Initialize AprilTag detection
        initAprilTag();
        
        // Wait for driver to press start
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
                
                // Handle driver controls for robot movement
                handleDriveControls();
                
                // Handle turret controls and tracking
                handleTurretControls();
                
                // Update telemetry
                updateTelemetry();
                
                sleep(20); // Small delay for stability
            }
        }
        
        // Disable the vision portal at the end
        visionPortal.close();
    }
    
    /**
     * Initialize all hardware components
     */
    private void initHardware() {
        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        
        // Initialize turret motor (comment out if using servo)
        turretMotor = hardwareMap.get(DcMotor.class, "turret_motor");
        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Initialize turret servo (uncomment if using servo instead of motor)
        // turretServo = hardwareMap.get(Servo.class, "turret_servo");
        // turretServo.setPosition(0.5); // Center position
        
        // Set drive motor directions (adjust based on your robot configuration)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Set zero power behavior for drive
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    /**
     * Initialize AprilTag detection
     */
    private void initAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
            .setDrawAxes(false)
            .setDrawCubeProjection(false)
            .setDrawTagOutline(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagProcessor.TagLibrary.getCenterStageTagLibrary())
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .build();
            
        // Create the vision portal
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(aprilTag)
            .build();
    }
    
    /**
     * Handle driver controls for robot movement
     */
    private void handleDriveControls() {
        double drive = -gamepad1.left_stick_y;  // Forward/backward
        double strafe = gamepad1.left_stick_x;   // Left/right
        double turn = gamepad1.right_stick_x;    // Rotation
        
        // Apply speed multiplier
        drive *= DRIVE_SPEED_MULTIPLIER;
        strafe *= DRIVE_SPEED_MULTIPLIER;
        turn *= DRIVE_SPEED_MULTIPLIER;
        
        // Calculate motor powers for mecanum drive
        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;
        
        // Normalize the values so no wheel power exceeds 100%
        double maxPower = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));
        
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }
        
        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    
    /**
     * Handle turret controls and tracking
     */
    private void handleTurretControls() {
        // Toggle turret tracking with Y button
        if (gamepad1.y) {
            turretTrackingActive = !turretTrackingActive;
            sleep(200); // Debounce
        }
        
        if (turretTrackingActive) {
            // Automatic turret tracking
            handleAutomaticTurretTracking();
        } else {
            // Manual turret control with right bumpers
            handleManualTurretControl();
        }
    }
    
    /**
     * Handle automatic turret tracking using AprilTags
     */
    private void handleAutomaticTurretTracking() {
        // Look for AprilTag detections
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        // Find the closest goal tag
        AprilTagDetection targetTag = findClosestGoalTag(currentDetections);
        
        if (targetTag != null) {
            // We found a target, calculate the angle error
            double bearingError = targetTag.ftcPose.bearing;
            
            // Only move turret if error is outside deadband
            if (Math.abs(bearingError) > TURRET_DEADBAND) {
                double turretSpeed = bearingError * TURRET_SPEED_GAIN;
                turretSpeed = Range.clip(turretSpeed, -MAX_TURRET_SPEED, MAX_TURRET_SPEED);
                
                // Control turret motor
                if (turretMotor != null) {
                    turretMotor.setPower(-turretSpeed); // Negative to correct direction
                }
                
                // Alternative: Control turret servo (uncomment if using servo)
                // if (turretServo != null) {
                //     double currentPos = turretServo.getPosition();
                //     double newPos = Range.clip(currentPos - (turretSpeed * 0.01), 0.0, 1.0);
                //     turretServo.setPosition(newPos);
                // }
                
                turretSearchTimer.reset(); // Reset search timer since we found a target
            } else {
                // Target is centered, stop turret
                if (turretMotor != null) {
                    turretMotor.setPower(0);
                }
            }
        } else {
            // No target found - search behavior
            handleTurretSearch();
        }
    }
    
    /**
     * Handle turret search when no target is found
     */
    private void handleTurretSearch() {
        // Slowly sweep back and forth to find targets
        if (turretSearchTimer.seconds() < 3.0) {
            // Search right
            if (turretMotor != null) {
                turretMotor.setPower(TURRET_SEARCH_SPEED);
            }
        } else if (turretSearchTimer.seconds() < 6.0) {
            // Search left
            if (turretMotor != null) {
                turretMotor.setPower(-TURRET_SEARCH_SPEED);
            }
        } else {
            // Reset search timer
            turretSearchTimer.reset();
        }
    }
    
    /**
     * Handle manual turret control
     */
    private void handleManualTurretControl() {
        double turretPower = 0;
        
        // Manual control with right bumpers
        if (gamepad1.right_bumper) {
            turretPower = 0.3; // Turn right
        } else if (gamepad1.left_bumper) {
            turretPower = -0.3; // Turn left
        }
        
        // Also allow fine control with right stick X when not using it for robot turning
        if (Math.abs(gamepad1.right_stick_x) > 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper) {
            turretPower = gamepad1.right_stick_x * 0.4;
        }
        
        if (turretMotor != null) {
            turretMotor.setPower(turretPower);
        }
        
        // Alternative servo control (uncomment if using servo)
        // if (turretServo != null && Math.abs(turretPower) > 0.1) {
        //     double currentPos = turretServo.getPosition();
        //     double newPos = Range.clip(currentPos + (turretPower * 0.02), 0.0, 1.0);
        //     turretServo.setPosition(newPos);
        // }
    }
    
    /**
     * Find the closest goal AprilTag from current detections
     */
    private AprilTagDetection findClosestGoalTag(List<AprilTagDetection> detections) {
        AprilTagDetection closestTag = null;
        double closestDistance = Double.MAX_VALUE;
        
        for (AprilTagDetection detection : detections) {
            // Check if this is a goal tag
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
    
    /**
     * Check if the given tag ID is a goal tag
     */
    private boolean isGoalTag(int tagId) {
        for (int goalId : GOAL_TAG_IDS) {
            if (tagId == goalId) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        telemetry.addData("Status", "TeleOp Running: %.1f seconds", runtime.seconds());
        telemetry.addData("Turret Tracking", turretTrackingActive ? "ACTIVE" : "MANUAL");
        
        if (turretMotor != null) {
            telemetry.addData("Turret Position", turretMotor.getCurrentPosition());
            telemetry.addData("Turret Power", "%.2f", turretMotor.getPower());
        }
        
        // Show current AprilTag detections
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
        
        telemetry.addData("Controls", "Y: Toggle tracking, Bumpers: Manual turret");
        telemetry.update();
    }
}
