/*
Copyright 2024 FIRST Tech Challenge Team 19564

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;


@Autonomous (name = "IMU")

public class OmarCompition extends LinearOpMode {
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor bl = null;
    DcMotor br = null;
    double intergral = 0;
    double kp = 0.5;
    double ki = 0.5;
    double kd = 0.5;
    ElapsedTime elapsedTime = new ElapsedTime();
    double previous = 0;
    private BNO055IMU imu;
    
    
    @Override
    public void runOpMode() {
        
            
        fl  = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        BNO055IMU.Parameters paramsIMU = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        waitForStart();
        
        move(1000, 1000, 0.2);
        
        move(-500, 500, 0.2);
        
        move(1000, 1000, 0.2);
        
        move(0, 250, 0.2);
        
        move(500, 500, 0.2);
        //reverse everthing to go back home
        move(-500, -500, 0.2);
        move(250, -250, 0.2);
        move(-1000, -1000, 0.2); 
        move(500, -500, 0.2);
        move(-1000, -1000, 0.2);
        
    }
    
     public void move(int lTarget, int rTarget, double speed){
         bl.setTargetPosition(lTarget);
         fl.setTargetPosition(lTarget);
         br.setTargetPosition(rTarget);
         fr.setTargetPosition(rTarget);
         
         
         fl.setMode(DcMotor.RunMode.Run_To_Position);
         fr.setMode(DcMotor.RunMode.Run_To_Position);
         bl.setMode(DcMotor.RunMode.Run_To_Position);
         br.setMode(DcMotor.RunMode.Run_To_Position);
         
         fl.setPower(speed);
         fr.setPower(speed);
         bl.setPower(speed);
         br.setPower(speed);
         
         while (opModeIsActive() && (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())){
             idle();
         }
     }
    public double PIDController(double target,double current){
             double currentTime = elapsedTime.time();
             double proportionalError = target - current;
             intergral += proportionalError * currentTime;
             double derivative = (current - previous) / (currentTime);
             previous = current;
             elapsedTime.reset();
             return proportionalError * kp + integral * ki + derivative * kd;
         }

 }
