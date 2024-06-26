
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Opmodejava")

public class TestAutonomous extends LinearOpMode {
     
     DcMotor motor0 = null;
     DcMotor motor1 = null;
     DcMotor motor2 = null;
     DcMotor motor3 = null;

    @Override
    public void runOpMode() {
        motor0 = hardwareMap.get(DcMotor.class, "backleft");
        motor1 = hardwareMap.get(DcMotor.class, "frontleft");
        motor2 = hardwareMap.get(DcMotor.class, "frontright");
        motor3 = hardwareMap.get(DcMotor.class, "backright");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        motor0.setPower(-.5);
        motor1.setPower(.5);
        motor2.setPower(-.5);
        motor3.setPower(.5);
        sleep(1000);
        motor0.setPower(-.5);
        motor1.setPower(.5);
        motor2.setPower(-.5);
        motor3.setPower(.5);
   
        
        
        
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
