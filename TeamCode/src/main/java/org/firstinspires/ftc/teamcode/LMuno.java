package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RoadRunner.util.Encoder;


//@Disabled
@Config
@TeleOp(name="LM1", group="AAALinear Opmode")
public class LMuno extends LinearOpMode {

    public static double maxspeed;

    public static double servodown = 0.23;
    public static double servoup = 0.5;
    public static double hanging = 0.495;
    public static double hanging2 = 0.57;
    public static double secured = 0;
    public static double release = 1;
    public static double jail = 1;
    public static double freedom = 0;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftback;
    private DcMotor rightback;
    private DcMotor leftfront;
    private DcMotor rightfront;
//    private Servo grab;
    private Servo pixeldrop;
    private Servo hanger;
    private Servo hanger2;
    private Servo luik;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    private Encoder leftEncoder;

    @Override
    public void runOpMode() {
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, ConfigurationName.leftEncoder));


        leftfront = hardwareMap.dcMotor.get(ConfigurationName.leftfront);
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftfront.setDirection(DcMotor.Direction.FORWARD);

        leftback = hardwareMap.dcMotor.get(ConfigurationName.leftback);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftback.setDirection(DcMotor.Direction.FORWARD);

        rightfront = hardwareMap.dcMotor.get(ConfigurationName.rightfront);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setDirection(DcMotor.Direction.REVERSE);

        rightback = hardwareMap.dcMotor.get(ConfigurationName.rightback);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        pixeldrop = hardwareMap.servo.get(ConfigurationName.pixeldrop);
        hanger = hardwareMap.servo.get(ConfigurationName.hanger);
        hanger2 = hardwareMap.servo.get(ConfigurationName.hanger2);
//        grab = hardwareMap.servo.get(ConfigurationName.grab);
        luik = hardwareMap.servo.get(ConfigurationName.luik);




        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        while (opModeIsActive()) {

            if(Math.abs(gamepad1.right_trigger) < 0.02) {
                maxspeed = 0.8;

            } else if (Math.abs(gamepad1.right_trigger) > 0.02) {
                maxspeed = (-0.4*gamepad1.right_trigger) +0.5;

            }

            if (gamepad1.dpad_down) {
                pixeldrop.setPosition(servodown);
            } else if (gamepad1.right_bumper) {
                pixeldrop.setPosition(servoup   );
            }

            if (gamepad1.dpad_up) {
                hanger.setPosition(hanging);
                hanger2.setPosition(hanging2);
            }

//            if (gamepad2.dpad_down) {
//                grab.setPosition(secured);
//            } else if (gamepad2.dpad_up) {
//                grab.setPosition(release);
//            }

            if (gamepad1.a) {
                luik.setPosition(jail);
            } else if (gamepad1.b)
                luik.setPosition(freedom);

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double head = angles.firstAngle;

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-head) - y * Math.sin(-head);
            double rotY = x * Math.sin(-head) + y * Math.cos(-head);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX + rx) / denominator) * maxspeed;
            double backLeftPower = ((rotY - rotX + rx) / denominator) * maxspeed;
            double frontRightPower = ((rotY - rotX - rx) / denominator) * maxspeed;
            double backRightPower = ((rotY + rotX - rx) / denominator) * maxspeed;


            leftfront.setPower(frontLeftPower);
            leftback.setPower(backLeftPower);
            rightfront.setPower(frontRightPower);
            rightback.setPower(backRightPower);

            telemetry.addData("ENCODERHOER", leftEncoder.getCurrentPosition());


            telemetry.addData("Motors", "leftfront (%.2f)", frontLeftPower);
            telemetry.addData("Motors", "leftback (%.2f)", backLeftPower);
            telemetry.addData("Motors", "rightfront (%.2f)", frontRightPower);
            telemetry.addData("Motors", "rightback (%.2f)", backRightPower);
            telemetry.addData("IMU", "Radians (%.2f)", head);
            telemetry.addData("Servo", pixeldrop.getPosition());
            telemetry.addData("Servo", hanger.getPosition());
            telemetry.addData("Servo", hanger2.getPosition());
//            telemetry.addData("Servo", grab.getPosition());
            telemetry.addData("Servo", luik.getPosition());
            telemetry.update();
        }
    }
}