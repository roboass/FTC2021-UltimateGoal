package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class UsefulFunctions extends LinearOpMode {
    public DcMotor frontleft, frontright, backleft, backright, launchMotor;
    public Servo launchServo, liftClawServo1, liftClawServo2, grabClawServo1, grabClawServo2, angleLaunchServo1, angleLaunchServo2;
    public OpenCvCamera phoneCam;
    public ImageDetector detector = new ImageDetector();

    public static double currentLaunchAngle = 0;
    public static int currentClawState = 0;

    public static double ticks_rev = 753.2;
    public static int gear_ratio = 2;
    public static int diameter_mm = 100;
    public static double diameter_in = 3.94;

    public int crticksfl, crticksfr, crticksbl, crticksbr;

    public Thread launchServoThread = new Thread() {
        public void run() {
            launchServo.setPosition(-0.17);
            try {
                Thread.sleep(1000);
                launchServo.setPosition(0.17);
                Thread.sleep(700);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };

    //public BNO055IMU gyro;
    public Orientation crtangle = new Orientation();

    public void Initialise() {
        launchMotor = hardwareMap.get(DcMotor.class, "launch_motor");
        frontleft = hardwareMap.get(DcMotor.class, "front_left");
        frontright = hardwareMap.get(DcMotor.class, "front_right");
        backleft = hardwareMap.get(DcMotor.class, "back_left");
        backright = hardwareMap.get(DcMotor.class, "back_right");

        launchServo = hardwareMap.get(Servo.class, "launch_servo");
        angleLaunchServo1 = hardwareMap.get(Servo.class, "angle_servo1");
        angleLaunchServo2 = hardwareMap.get(Servo.class, "angle_servo2");

        liftClawServo1 = hardwareMap.get(Servo.class, "lcs1");
        liftClawServo2 = hardwareMap.get(Servo.class, "lcs2");
        /*grabClawServo1 = hardwareMap.get(Servo.class, "gcs1");
        grabClawServo2 = hardwareMap.get(Servo.class, "gcs2");*/

        SwitchMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        angleLaunchServo1.setDirection(Servo.Direction.REVERSE);
        angleLaunchServo2.setDirection(Servo.Direction.FORWARD);
        launchServo.setDirection(Servo.Direction.FORWARD);
        launchMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        liftClawServo1.setDirection(Servo.Direction.REVERSE);
        liftClawServo2.setDirection(Servo.Direction.FORWARD);

        currentLaunchAngle = 0;
        currentClawState = 0;

        AddToLaunchAngle(31);

        //Partea drepta mere in fata
        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);
        while (!gyro.isGyroCalibrated() && opModeIsActive())
        {
            telemetry.addData("IMU is calibrating!", "Please wait.");
            telemetry.update();
        }*/

        telemetry.addData("Init is done", "Press start");
        telemetry.update();
    }

    /*Asemanator cu functiile MoveSideMM si MoveFWBKMM merge-uite.
     * Ia ca parametri cat sa se miste pe axa x SAU pe axa y (in mm).
     * Una din axe trebe sa fie 0 altfel nu stiu ce se intampla
     */
    public void AutonomousMove(double x_mm, double y_mm) {
        double motorPower = 1;
        int sideOrFront;

        int ticksToMove_x = mm_to_ticks(x_mm);
        int ticksToMove_y = mm_to_ticks(y_mm);
        int ticksToMove = (ticksToMove_x == 0 ? ticksToMove_y : ticksToMove_x);
        sideOrFront = (x_mm != 0 ? 1 : -1);

        UpdateTicks();
        int trgtfl, trgtfr, trgtbl, trgtbr;
        trgtfl = crticksfl + ticksToMove;
        trgtfr = crticksfr - sideOrFront * ticksToMove;
        trgtbl = crticksbl + sideOrFront * ticksToMove;
        trgtbr = crticksbr - ticksToMove;

        SwitchMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setTargetPosition(trgtfl);
        frontright.setTargetPosition(trgtfr);
        backleft.setTargetPosition(trgtbl);
        backright.setTargetPosition(trgtbr);
        SwitchMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        ApplyMotorValues(new MotorValues(motorPower));

        while ((frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) && opModeIsActive()) {
            UpdateTicks();
            UpdateOrientation();
        }
        ApplyMotorValues(new MotorValues(0));
        UpdateTicks();
        UpdateOrientation();
    }

    /*Functia care controleaza miscarea in TeleOp.
     * Citeste din gamepad1, nu are parametri*/
    public void TeleOpDrive() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;

        double power_fl = x - y + rotation;
        double power_fr = x + y + rotation;
        double power_bl = -x - y + rotation;
        double power_br = -x + y + rotation;

        MotorValues motorValues = new MotorValues(power_fl, power_fr, power_bl, power_br, 0.5);

        if (gamepad1.left_bumper) motorValues.SlowMode();

        motorValues.NormaliseValues();
        ApplyMotorValues(motorValues);
    }

    public void SwitchMotorModes(DcMotor.RunMode x) {
        frontleft.setMode(x);
        frontright.setMode(x);
        backleft.setMode(x);
        backright.setMode(x);

        while ((frontleft.getMode() != x || frontright.getMode() != x || backleft.getMode() != x || backright.getMode() != x) && opModeIsActive())
            ;
    }

    public double in_to_mm(double x) {
        return 25.4 * x;
    }

    public int mm_to_ticks(double x) {
        return (int) (((ticks_rev * x) / (diameter_mm * Math.PI)) * gear_ratio);
    }

    public void ApplyMotorValues(MotorValues motorValues) {
        frontleft.setPower(motorValues.fl);
        frontright.setPower(motorValues.fr);
        backleft.setPower(motorValues.bl);
        backright.setPower(motorValues.br);
    }

    public void UpdateTicks() {
        crticksfl = frontleft.getCurrentPosition();
        crticksfr = frontright.getCurrentPosition();
        crticksbl = backleft.getCurrentPosition();
        crticksbr = backright.getCurrentPosition();

        UpdateOrientation();

        telemetry.addData("Ticks Front Left:", crticksfl);
        telemetry.addData("Ticks Front Right:", crticksfr);
        telemetry.addData("Ticks Back Left:", crticksbl);
        telemetry.addData("Ticks Back Right:", crticksbr);
        telemetry.addData("Heading:", crtangle.firstAngle);
        telemetry.update();
    }

    public void UpdateOrientation() {
        //crtangle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void InitialiseVision() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
    }

    public void GrabClawState(boolean state) {
        /*if(state)
        {
           grabClawServo1.setPosition(0.15);
           grabClawServo2.setPosition(0.15);
        }
        else
        {
            grabClawServo1.setPosition(-0.15);
            grabClawServo2.setPosition(-0.15);
        }*/
    }

    public void LiftClawState(int state) ///0 - deasupra lansator, 1 - tinut wobble goal, 2 - apucat wobble, 3 - inel
    {
        if (state < 0 || state > 3) return;
        double[] values = new double[]{0.07, 0.3, 0.7, 0.8};

        liftClawServo1.setPosition(values[state]);// - values[currentClawState]);
        liftClawServo2.setPosition(values[state]);// - values[currentClawState]);
        telemetry.addData("servo position", values[state] - values[currentClawState]);
        currentClawState = state;
    }

    public void AddToLaunchAngle(double angle)
    {
        if(currentLaunchAngle + angle <= 90) {
            angleLaunchServo1.setPosition(anglesToPercent(-angle));
            angleLaunchServo2.setPosition(anglesToPercent(-angle));
            currentLaunchAngle += angle;
        }
    }

    public double anglesToPercent(double x)
    {
        return x / 180;
    }
    @Override
    public void runOpMode () throws InterruptedException {
    }
}
