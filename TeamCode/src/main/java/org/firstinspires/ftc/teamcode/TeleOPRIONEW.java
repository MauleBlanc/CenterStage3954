package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Disabled
@TeleOp(name="RObot in onw week", group="AAALinear Opmode")
public class TeleOPRIONEW extends LinearOpMode {

    public static double maxspeed = 0.5;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftback;
    private DcMotor rightback;
    private DcMotor leftfront;
    private DcMotor rightfront;

    private DcMotor MotorArmpie;
    private DcMotor Rollers;

    private Servo pixeldrop;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() {

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

        MotorArmpie = hardwareMap.dcMotor.get(ConfigurationName.motorarmpie);
        MotorArmpie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorArmpie.setDirection(DcMotorSimple.Direction.FORWARD);

        Rollers = hardwareMap.dcMotor.get(ConfigurationName.Rollers);
        Rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rollers.setDirection(DcMotorSimple.Direction.FORWARD);

        pixeldrop = hardwareMap.servo.get(ConfigurationName.pixeldrop);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.useExternalCrystal = true;


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        while (opModeIsActive()) {
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
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            leftfront.setPower(frontLeftPower);
            leftback.setPower(backLeftPower);
            rightfront.setPower(frontRightPower);
            rightback.setPower(backRightPower);


            if(gamepad1.dpad_up) {
                MotorArmpie.setTargetPosition(Globalvalues.armpup);
                MotorArmpie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.left_bumper) {
                Rollers.setPower(Globalvalues.rolling);
            }
            if (gamepad1.dpad_down) {
                pixeldrop.setPosition(80);
            }

            telemetry.addData("Armpie", MotorArmpie.getCurrentPosition());
            telemetry.addData("Armpie", MotorArmpie.getTargetPosition());
            telemetry.addData("Armpie", MotorArmpie.getPower());
            telemetry.addData("Rollin", Rollers.getCurrentPosition());
            telemetry.addData("Rollin", Rollers.getTargetPosition());
            telemetry.addData("Rollin", Rollers.getPower());

            telemetry.addData("Motors", "leftfront (%.2f)", frontLeftPower);
            telemetry.addData("Motors", "leftback (%.2f)", backLeftPower);
            telemetry.addData("Motors", "rightfront (%.2f)", frontRightPower);
            telemetry.addData("Motors", "rightback (%.2f)", backRightPower);
            telemetry.addData("IMU", "Radians (%.2f)", head);
            telemetry.update();
        }
    }
}