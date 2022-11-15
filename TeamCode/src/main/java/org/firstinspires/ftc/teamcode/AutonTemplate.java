package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;

@TeleOp
public class AutonTemplate extends LinearOpMode {
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
    private ElapsedTime runtime = new ElapsedTime();

    private void drive(int x, int y, int r) {
        double yd = -0.4 * y;
        double xd = 0.4 * x * 1.1;
        double rd = 0.3 * r;
        double denominator = Math.max(Math.abs(yd) + Math.abs(xd) + Math.abs(rd), 1);
        double frontLeftPower = (yd + xd + rd) / denominator;
        double backLeftPower = (yd - xd + rd) / denominator;
        double frontRightPower = (yd - xd - rd) / denominator;
        double backRightPower = (yd + xd - rd) / denominator;
        leftFrontDrive.setPower(-frontLeftPower);
        leftBackDrive.setPower(-backLeftPower);
        rightFrontDrive.setPower(-frontRightPower);
        rightBackDrive.setPower(-backRightPower);
    }

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(864, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        //HARDWARE MAPPING HERE etc.
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
        s6.setPosition(0.4);
        int mirar = 0;
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        mirar = tag.id;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }


        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            drive(1, 0, 0);
        }
        if (mirar == 1){
            while (opModeIsActive() && (runtime.seconds() < 1.5)) {
                drive(0, -1, 0);
            }
    }else if(mirar ==3)
            while (opModeIsActive() && (runtime.seconds() < 1.5)) {
                drive(0,1,0);
            }
    {

    }

}

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}