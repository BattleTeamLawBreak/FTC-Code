import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(name="ALI")
public class SEAMAutonAli extends LinearOpMode {
    Servo hookL;
    Servo hookR;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor wrist;
    DcMotor leftShoulder;
    DcMotor rightShoulder;
    BNO055IMU imu;

@Override
    public void runOpMode() {
      hookL = hardwareMap.get(Servo.class, "hookL");
      hookR = hardwareMap.get(Servo.class, "hookR");
    /*  backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
      backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
      frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
      frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
      wrist = hardwareMap.get(DcMotor.class, "wrist");
      leftShoulder = hardwareMap.get(DcMotor.class, "leftShoulder");
      rightShoulder = hardwareMap.get(DcMotor.class, "rightShoulder");
      imu = hardwareMap.get(BNO055IMU.class, "imu");
      */
      // Put initialization blocks here
      waitForStart();
      // Put run blocks here
      while (opModeIsActive()) {
          if (gamepad1.y){
              
              hookL.setPosition(0);
              
          } 
          
          else if(gamepad1.b){
              hookL.setPosition(0.5);
              
          }
          
          else if(gamepad1.a){
              
              hookL.setPosition(1);
              
          }
          
       

      }
    }
    
}
