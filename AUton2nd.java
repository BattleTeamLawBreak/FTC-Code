    package org.firstinspires.ftc.teamcode;
    
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.Servo;
    
    @Autonomous//(name = "PowerAuton", group = "LinearOpMode")
    public class AUTON2ND extends LinearOpMode {
        private DcMotor frontLeft, frontRight, backLeft, backRight;
        private DcMotor leftArm, rightArm;
        private Servo servo3;  // Wrist
        private Servo servo2;  // Claw
    
        @Override
        public void runOpMode() {
            // Initialize hardware
            frontLeft = hardwareMap.get(DcMotor.class, "rearleft");
            frontRight = hardwareMap.get(DcMotor.class, "rearright");
            backLeft = hardwareMap.get(DcMotor.class, "frontright");
            backRight = hardwareMap.get(DcMotor.class, "frontleft");
            leftArm = hardwareMap.get(DcMotor.class, "motor3");
            rightArm = hardwareMap.get(DcMotor.class, "motor4");
            servo3 = hardwareMap.get(Servo.class, "servo3");
            servo2 = hardwareMap.get(Servo.class, "servo2");
    
            // Set motor directions
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);
            leftArm.setDirection(DcMotor.Direction.FORWARD);
            rightArm.setDirection(DcMotor.Direction.REVERSE);
            
            //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Reset and configure arm encoders
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
            // Set initial servo position
            servo2.setPosition(.1);  // Claw
            servo3.setPosition(0.26);  // Wrist
    
            telemetry.addData("Status", "Initialized");
            telemetry.update();
    
            waitForStart();
    
            if (opModeIsActive()) {
             //   Move arm up and hold position
             
                moveArmToPosition(-400, 0.5);
                servo3.setPosition(0.26);
                moveForward(-825, 0.35);
                sleep(650);
                moveArmToPosition(-470, 0.5);
                sleep(500);
                moveBackward(-175, 0.5);
                sleep(250);
                servo2.setPosition(0.8);
                moveBackward(-120, 0.5);
            
                
                
                
    
                // Open the claw
              // Release the claw
               
    
    
                // Strafe left (approx 2 feet)
                strafeLeft(1300, 0.30);  // Adjust tick value as needed
                
                moveArmToPosition(-1630, 0.5);  // Adjusted arm position
                holdArmPosition();
                servo3.setPosition(0.6);
                servo2.setPosition(.8);
                sleep(300);
                moveBackward(-155, 0.5);
                servo2.setPosition(.6);
                
                sleep(250);
                servo2.setPosition(.6);
                sleep(500);
                moveArmToPosition(-385, 0.5);
                holdArmPosition();
                servo3.setPosition(.35);
    
                strafeLeft(-1350, 0.30);
                sleep(500);
                moveForward(-350, 0.35);
                sleep(500);
                moveArmToPosition(-485, 0.5);//less=less
                sleep(275);
                moveBackward(-250, 0.35);
                servo2.setPosition(0.8);
                sleep(250);
                strafeLeft(1000, 0.30);
                moveForward(-600, 0.5);
                sleep(500);
                strafeLeft(340, 0.35);
                moveBackward(-1080, 0.5);
                 sleep(500);
                  strafeLeft(-300, 0.35);
                 moveForward(-1500, 0.5);
                 strafeLeft(600, 0.35);
                 moveBackward(-1050, 0.5);
                sleep(500);
                 strafeLeft(-300, 0.35);
                 moveForward(-1500, 0.5);
                 strafeLeft(600, 0.35);
                 moveBackward(-1050, 0.5);
                
                
            }
        }
    
        private void moveForward(int ticks, double power) {
            setTargetPosition(ticks);
            setMotorPower(power);
            waitForMotors();
        }
    
        private void moveBackward(int ticks, double power) {
            setTargetPosition(-ticks);
            setMotorPower(power);
            waitForMotors();
        }
    
        private void strafeLeft(int ticks, double power) {
            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - ticks);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + ticks);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
            backRight.setTargetPosition(backRight.getCurrentPosition() - ticks);
    
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
            setMotorPower(power);
            waitForMotors();
        }
    
        private void setTargetPosition(int ticks) {
            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + ticks);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
            backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);
    
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    
        private void setMotorPower(double power) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }
    
        private void waitForMotors() {
            while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                telemetry.addData("Moving", "Motors running...");
                telemetry.update();
            }
            stopMotors();
        }
    
        private void stopMotors() {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    
        private void moveArmToPosition(int position, double power) {
            leftArm.setTargetPosition(position);
            rightArm.setTargetPosition(position);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftArm.setPower(power);
            rightArm.setPower(power);
    
            while (opModeIsActive() && leftArm.isBusy() && rightArm.isBusy()) {
                telemetry.addData("Arm", "Moving to position %d", position);
                telemetry.update();
            }
            holdArmPosition();
        }
    
        private void holdArmPosition() {
            leftArm.setPower(0.1);
            rightArm.setPower(0.1);
        }
    }
