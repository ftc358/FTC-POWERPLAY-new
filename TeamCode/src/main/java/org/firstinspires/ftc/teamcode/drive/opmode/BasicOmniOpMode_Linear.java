package org.firstinspires.ftc.teamcode.drive.opmode;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@TeleOp(name="mcnanum drive training", group="Linear Opmode")
public class BasicOmniOpMode_Linear extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private CRServo s1 = null;
    private CRServo s2 = null;
    private Servo s3 = null;
    private Servo s4 = null;
    private DcMotor liftmotorright = null;
    private DcMotor liftmotorleft = null;
    private Servo s5 = null;
    private Servo s6 = null;
    private Servo s7 = null;
    private Servo s8 = null;


    @Override
    public void runOpMode() {

        s1 = hardwareMap.crservo.get("s1"); //intake
        s2 = hardwareMap.crservo.get("s2");
        s3 = hardwareMap.servo.get("s3"); //claw
        s4 = hardwareMap.servo.get("s4");
        s5 = hardwareMap.servo.get("sl"); //claw neck
        s6 = hardwareMap.servo.get("sr");
        s7 = hardwareMap.servo.get("s7"); //thing that goes in and out
        s8 = hardwareMap.servo.get("s8");


        leftFrontDrive = hardwareMap.dcMotor.get("lf");
        leftBackDrive = hardwareMap.dcMotor.get("lb");
        rightFrontDrive = hardwareMap.dcMotor.get("rf");
        rightBackDrive = hardwareMap.dcMotor.get("rb");

        liftmotorleft = hardwareMap.dcMotor.get("m1");
        liftmotorright = hardwareMap.dcMotor.get("m2");

        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotorleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotorright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotorleft.setDirection(DcMotor.Direction.REVERSE);
        s3.setPosition(0.4);
        s4.setPosition(0.1);


        boolean clamp = false;
        boolean clamplast = false;
        boolean toggle = false;
        boolean intake = false;
        boolean in = false;
        boolean inlast = false;
        boolean toggle2 = false;
        boolean toggle3 = false;
        boolean intakelast = false;
        boolean toggle1 = false;
        boolean neck = false;
        boolean necklast = false;
        //s5.setPosition(0.4);
        s6.setPosition(0.4);
        waitForStart();
        while (opModeIsActive()) {
            double y = -0.5*gamepad1.left_stick_y;
            double x = 0.3*gamepad1.left_stick_x * 1.1;
            double rx = 0.3*gamepad1.right_stick_x;

            clamplast = clamp;
            clamp = gamepad1.right_bumper;
            if (clamplast == false && clamp == true){
                toggle = !toggle;
            }
            if(toggle){
                s3.setPosition(0.3);
                s4.setPosition(0.2);
            }else{
                s3.setPosition(0.4);
                s4.setPosition(0.1);
            }
            /*
            inlast = in;
            in = gamepad1.x;
            if (inlast == false && in == true){
                toggle2 = !toggle2;
            }
            if(toggle2){
                s7.setPosition(0.2);
                s8.setPosition(0.8);
            }else{
                s7.setPosition(0.8);
                s8.setPosition(0.2);
            }
             */
            necklast = neck;
            neck = gamepad1.left_bumper;
            if (necklast == false && neck == true){
                toggle3 = !toggle3;
            }
            if(toggle3){
                telemetry.addLine("fuck1");
                telemetry.update();
                //s5.setPosition(0.4);
               s6.setPosition(0.1);
            }else{
                //s5.setPosition(0.1);
                s6.setPosition(0.4);
            }

            intakelast = intake;
            intake = gamepad1.a;
            if (intakelast == false && intake == true){
                toggle1 = !toggle1;
            }
            if(toggle1){
                telemetry.addLine("fuck1");
                telemetry.update();
                s1.setPower(1);
                s2.setPower(0);
            }else{
                s1.setPower(0);
                s2.setPower(1);
            }


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            if(gamepad1.right_stick_y > 0){
                liftmotorleft.setPower(0.1*gamepad1.right_stick_y);
                liftmotorright.setPower(0.1*gamepad1.right_stick_y);
            }else{
                liftmotorleft.setPower(gamepad1.right_stick_y);
                liftmotorright.setPower(gamepad1.right_stick_y);
            }

            leftFrontDrive.setPower(-frontLeftPower);
            leftBackDrive.setPower(-backLeftPower);
            rightFrontDrive.setPower(-frontRightPower);
            rightBackDrive.setPower(-backRightPower);

        }
    }


        }


