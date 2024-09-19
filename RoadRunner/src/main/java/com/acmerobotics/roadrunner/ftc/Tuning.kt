package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.TankKinematics
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeProfile
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.constantProfile
import com.google.gson.annotations.SerializedName
import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.SerialNumber
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min

class MidpointTimer {
    private val beginTs = System.nanoTime()
    private var lastTime: Long = 0

    fun seconds(): Double {
        return 1e-9 * (System.nanoTime() - beginTs)
    }

    fun addSplit(): Double {
        val time = System.nanoTime() - beginTs
        val midTimeSecs = 0.5e-9 * (lastTime + time)
        lastTime = time
        return midTimeSecs
    }
}

private class MutableSignal(
        val times: MutableList<Double> = mutableListOf(),
        val values: MutableList<Double> = mutableListOf()
)

enum class DriveType {
    @SerializedName("mecanum")
    MECANUM,
    @SerializedName("tank")
    TANK
}

private fun unwrap(e: Encoder): RawEncoder =
        when (e) {
            is OverflowEncoder -> e.encoder
            is RawEncoder -> e
        }

fun interface FeedforwardFactory {
    fun make(): MotorFeedforward
}

class DriveView(
        val type: DriveType,
        val inPerTick: Double,
        val maxVel: Double,
        val minAccel: Double,
        val maxAccel: Double,
        val lynxModules: List<LynxModule>,
        // ordered front to rear
        val leftMotors: List<DcMotorEx>,
        val rightMotors: List<DcMotorEx>,
        // invariant: (leftEncs.isEmpty() && rightEncs.isEmpty()) ||
        //                  (parEncs.isEmpty() && perpEncs.isEmpty())
        leftEncs: List<Encoder>,
        rightEncs: List<Encoder>,
        parEncs: List<Encoder>,
        perpEncs: List<Encoder>,
        val imu: LazyImu,
        val voltageSensor: VoltageSensor,
        val feedforwardFactory: FeedforwardFactory,
) {
    val motors = leftMotors + rightMotors

    val leftEncs = leftEncs.map(::unwrap)
    val rightEncs = rightEncs.map(::unwrap)
    val parEncs = parEncs.map(::unwrap)
    val perpEncs = perpEncs.map(::unwrap)

    val forwardEncsWrapped = leftEncs + rightEncs + parEncs
    val forwardEncs = forwardEncsWrapped.map(::unwrap)

    init {
        require((leftEncs.isEmpty() && rightEncs.isEmpty()) || (parEncs.isEmpty() && perpEncs.isEmpty()))
    }

    fun setBulkCachingMode(mode: LynxModule.BulkCachingMode) {
        for (m in lynxModules) {
            m.bulkCachingMode = mode
        }
    }

    fun resetAndBulkRead(t: MidpointTimer): Map<SerialNumber, Double> {
        val times = mutableMapOf<SerialNumber, Double>()
        for (m in lynxModules) {
            m.clearBulkCache()

            t.addSplit()
            m.getBulkData()
            times[m.serialNumber] = t.addSplit()
        }
        return times
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
        when (type) {
            DriveType.MECANUM -> {
                val wheelPowers = MecanumKinematics(1.0).inverse(PoseVelocity2dDual.constant<Time>(powers, 1))
                val maxPowerMag = wheelPowers.all().maxOfOrNull { it.value().absoluteValue }!!
                val divisor = max(1.0, maxPowerMag)

                leftMotors[0].power = wheelPowers.leftFront.value() / divisor
                leftMotors[1].power = wheelPowers.leftBack.value() / divisor
                rightMotors[0].power = wheelPowers.rightFront.value() / divisor
                rightMotors[1].power = wheelPowers.rightBack.value() / divisor
            }

            DriveType.TANK -> {
                val wheelPowers = TankKinematics(2.0).inverse(PoseVelocity2dDual.constant<Time>(powers, 1))
                val maxPowerMag = wheelPowers.all().maxOfOrNull { it.value().absoluteValue }!!
                val divisor = max(1.0, maxPowerMag)

                for (m in leftMotors) {
                    m.power = wheelPowers.left.value() / divisor
                }
                for (m in rightMotors) {
                    m.power = wheelPowers.right.value() / divisor
                }
            }
        }
    }
}

interface DriveViewFactory {
    fun make(h: HardwareMap): DriveView
}

// designed for manual bulk caching
private fun recordEncoderData(e: Encoder, ts: Map<SerialNumber, Double>, ps: MutableSignal, vs: MutableSignal) {
    val sn = (e.controller as LynxDcMotorController).serialNumber
    val p = e.getPositionAndVelocity()

    ps.times.add(ts[sn]!!)
    ps.values.add(p.position.toDouble())

    vs.times.add(ts[sn]!!)
    vs.values.add(p.velocity.toDouble())
}

class AngularRampLogger(val dvf: DriveViewFactory) : LinearOpMode() {
    companion object {
        @JvmField
        var POWER_PER_SEC = 0.1
        @JvmField
        var POWER_MAX = 0.9
    }

    fun power(seconds: Double) = min(POWER_PER_SEC * seconds, POWER_MAX)

    override fun runOpMode() {
        val view = dvf.make(hardwareMap)
        view.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)

        val data = object {
            val type = view.type
            val leftPowers = view.leftMotors.map { MutableSignal() }
            val rightPowers = view.rightMotors.map { MutableSignal() }
            val voltages = MutableSignal()
            val leftEncPositions = view.leftEncs.map { MutableSignal() }
            val rightEncPositions = view.rightEncs.map { MutableSignal() }
            val parEncPositions = view.parEncs.map { MutableSignal() }
            val perpEncPositions = view.perpEncs.map { MutableSignal() }
            val leftEncVels = view.leftEncs.map { MutableSignal() }
            val rightEncVels = view.rightEncs.map { MutableSignal() }
            val parEncVels = view.parEncs.map { MutableSignal() }
            val perpEncVels = view.perpEncs.map { MutableSignal() }
            val angVels = listOf(MutableSignal(), MutableSignal(), MutableSignal())
        }

        waitForStart()

        val t = MidpointTimer()
        while (opModeIsActive()) {
            for (i in view.leftMotors.indices) {
                val power = -power(t.seconds())
                view.leftMotors[i].power = power

                val s = data.leftPowers[i]
                s.times.add(t.addSplit())
                s.values.add(power)
            }

            for (i in view.rightMotors.indices) {
                val power = power(t.seconds())
                view.rightMotors[i].power = power

                val s = data.rightPowers[i]
                s.times.add(t.addSplit())
                s.values.add(power)
            }

            data.voltages.values.add(view.voltageSensor.voltage)
            data.voltages.times.add(t.addSplit())

            val encTimes = view.resetAndBulkRead(t)

            for (i in view.leftEncs.indices) {
                recordEncoderData(
                        view.leftEncs[i],
                        encTimes,
                        data.leftEncPositions[i],
                        data.leftEncVels[i]
                )
            }

            for (i in view.rightEncs.indices) {
                recordEncoderData(
                        view.rightEncs[i],
                        encTimes,
                        data.rightEncPositions[i],
                        data.rightEncVels[i]
                )
            }

            for (i in view.parEncs.indices) {
                recordEncoderData(
                        view.parEncs[i],
                        encTimes,
                        data.parEncPositions[i],
                        data.parEncVels[i]
                )
            }

            for (i in view.perpEncs.indices) {
                recordEncoderData(
                        view.perpEncs[i],
                        encTimes,
                        data.perpEncPositions[i],
                        data.perpEncVels[i]
                )
            }

            t.addSplit()
            // Use degrees here to work around https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070
            val av = view.imu.get().getRobotAngularVelocity(AngleUnit.DEGREES)
            val time = t.addSplit()

            data.angVels[0].times.add(time)
            data.angVels[1].times.add(time)
            data.angVels[2].times.add(time)

            data.angVels[0].values.add(Math.toRadians(av.xRotationRate.toDouble()))
            data.angVels[1].values.add(Math.toRadians(av.yRotationRate.toDouble()))
            data.angVels[2].values.add(Math.toRadians(av.zRotationRate.toDouble()))
        }

        for (m in view.motors) {
            m.power = 0.0
        }

        TuningFiles.save(TuningFiles.FileType.ANGULAR_RAMP, data)
    }
}

private fun avgPos(es: List<Encoder>) = es.sumOf { it.getPositionAndVelocity().position.toDouble() } / es.size

class ForwardPushTest(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
        val view = dvf.make(hardwareMap)

        for (m in view.motors) {
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }

        waitForStart()

        val initAvgPos = avgPos(view.forwardEncs)
        while (opModeIsActive()) {
            telemetry.addData("ticks traveled", avgPos(view.forwardEncs) - initAvgPos)
            telemetry.update()
        }
    }
}

class ForwardRampLogger(val dvf: DriveViewFactory) : LinearOpMode() {
    companion object {
        @JvmField
        var POWER_PER_SEC = 0.1
        @JvmField
        var POWER_MAX = 0.9
    }

    fun power(seconds: Double) = min(POWER_PER_SEC * seconds, POWER_MAX)

    override fun runOpMode() {
        val view = dvf.make(hardwareMap)
        require(view.perpEncs.isNotEmpty()) {
            "Only run this op mode if you're using dead wheels."
        }

        view.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)

        val data = object {
            val type = view.type
            val powers = view.motors.map { MutableSignal() }
            val voltages = MutableSignal()
            val forwardEncPositions = view.forwardEncs.map { MutableSignal() }
            val forwardEncVels = view.forwardEncs.map { MutableSignal() }
        }

        waitForStart()

        val t = MidpointTimer()
        while (opModeIsActive()) {
            for (i in view.motors.indices) {
                val power = power(t.seconds())
                view.motors[i].power = power

                val s = data.powers[i]
                s.times.add(t.addSplit())
                s.values.add(power)
            }

            data.voltages.values.add(view.voltageSensor.voltage)
            data.voltages.times.add(t.addSplit())

            val encTimes = view.resetAndBulkRead(t)

            for (i in view.forwardEncs.indices) {
                recordEncoderData(
                        view.forwardEncs[i],
                        encTimes,
                        data.forwardEncPositions[i],
                        data.forwardEncVels[i]
                )
            }
        }

        for (m in view.motors) {
            m.power = 0.0
        }

        TuningFiles.save(TuningFiles.FileType.FORWARD_RAMP, data)
    }
}

class LateralRampLogger(val dvf: DriveViewFactory) : LinearOpMode() {
    companion object {
        @JvmField
        var POWER_PER_SEC = 0.1
        @JvmField
        var POWER_MAX = 0.9
    }

    fun power(seconds: Double) = min(POWER_PER_SEC * seconds, POWER_MAX)

    override fun runOpMode() {
        val view = dvf.make(hardwareMap)
        view.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)

        require(view.type == DriveType.MECANUM) {
            "Only mecanum drives should run this op mode."
        }

        val data = object {
            val type = view.type
            val frontLeftPower = MutableSignal()
            val backLeftPower = MutableSignal()
            val frontRightPower = MutableSignal()
            val backRightPower = MutableSignal()
            val voltages = MutableSignal()
            val perpEncPositions = view.perpEncs.map { MutableSignal() }
            val perpEncVels = view.perpEncs.map { MutableSignal() }
        }

        waitForStart()

        val t = MidpointTimer()

        fun setMotorPower(m: DcMotorEx, sign: Int, signal: MutableSignal) {
            val power = sign * power(t.seconds())
            m.power = power

            signal.times.add(t.addSplit())
            signal.values.add(power)
        }

        while (opModeIsActive()) {
            setMotorPower(view.leftMotors[0], -1, data.frontLeftPower)
            setMotorPower(view.rightMotors[0], +1, data.frontRightPower)
            setMotorPower(view.leftMotors[1], +1, data.backLeftPower)
            setMotorPower(view.rightMotors[1], -1, data.backRightPower)

            data.voltages.values.add(view.voltageSensor.voltage)
            data.voltages.times.add(t.addSplit())

            val encTimes = view.resetAndBulkRead(t)

            for (i in view.perpEncs.indices) {
                recordEncoderData(
                        view.perpEncs[i],
                        encTimes,
                        data.perpEncPositions[i],
                        data.perpEncVels[i]
                )
            }
        }

        for (m in view.motors) {
            m.power = 0.0
        }

        TuningFiles.save(TuningFiles.FileType.LATERAL_RAMP, data)
    }
}

fun lateralSum(view: DriveView): Double {
    return 0.25 * (
            -view.leftEncs[0].getPositionAndVelocity().position
                    +view.leftEncs[1].getPositionAndVelocity().position
                    -view.rightEncs[1].getPositionAndVelocity().position
                    +view.rightEncs[0].getPositionAndVelocity().position)
}

class LateralPushTest(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
        val view = dvf.make(hardwareMap)

        require(view.type == DriveType.MECANUM) {
            "Only mecanum drives should run this op mode."
        }
        require(view.parEncs.isEmpty() && view.perpEncs.isEmpty()) {
            "Do not run this op mode if using dead wheels."
        }

        for (m in view.motors) {
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }

        waitForStart()

        val initLateralSum = lateralSum(view)
        while (opModeIsActive()) {
            telemetry.addData("ticks traveled", lateralSum(view) - initLateralSum)
            telemetry.update()
        }
    }
}

class ManualFeedforwardTuner(val dvf: DriveViewFactory) : LinearOpMode() {
    companion object {
        @JvmField
        var DISTANCE = 64.0
    }

    enum class Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    override fun runOpMode() {
        val telemetry = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)

        val view = dvf.make(hardwareMap)
        val profile = TimeProfile(constantProfile(
                DISTANCE, 0.0, view.maxVel, view.minAccel, view.maxAccel).baseProfile)

        var mode = Mode.TUNING_MODE

        telemetry.addLine("Ready!")
        telemetry.update()
        telemetry.clearAll()

        waitForStart()

        if (isStopRequested) return

        var movingForwards = true
        var startTs = System.nanoTime() / 1e9

        while (!isStopRequested) {
            telemetry.addData("mode", mode)

            when (mode) {
                Mode.TUNING_MODE -> {
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE
                    }

                    for (i in view.forwardEncsWrapped.indices) {
                        val v = view.forwardEncsWrapped[i].getPositionAndVelocity().velocity
                        telemetry.addData("v$i", view.inPerTick * v)
                    }

                    val ts = System.nanoTime() / 1e9
                    val t = ts - startTs
                    if (t > profile.duration) {
                        movingForwards = !movingForwards
                        startTs = ts
                    }

                    var v = profile[t].drop(1)
                    if (!movingForwards) {
                        v = v.unaryMinus()
                    }
                    telemetry.addData("vref", v[0])

                    val power = view.feedforwardFactory.make().compute(v) / view.voltageSensor.voltage
                    view.setDrivePowers(PoseVelocity2d(Vector2d(power, 0.0), 0.0))
                }
                Mode.DRIVER_MODE -> {
                    if (gamepad1.b) {
                        mode = Mode.TUNING_MODE
                        movingForwards = true
                        startTs = System.nanoTime() / 1e9
                    }

                    view.setDrivePowers(PoseVelocity2d(
                            Vector2d(
                                    -gamepad1.left_stick_y.toDouble(),
                                    -gamepad1.left_stick_x.toDouble()
                            ),
                            -gamepad1.right_stick_x.toDouble()
                    ))
                }
            }

            telemetry.update()
        }
    }
}

class MecanumMotorDirectionDebugger(val dvf: DriveViewFactory) : LinearOpMode() {
    companion object {
        @JvmField
        var MOTOR_POWER = 0.7
    }

    override fun runOpMode() {
        val view = dvf.make(hardwareMap)

        require(view.type == DriveType.MECANUM) {
            "Only mecanum drives should run this op mode."
        }

        val telemetry = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)

        telemetry.addLine("Press play to begin the debugging op mode")
        telemetry.update()

        waitForStart()

        if (isStopRequested) return

        telemetry.clearAll()
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)

        while (opModeIsActive()) {
            telemetry.addLine("Press each button to turn on its respective motor")
            telemetry.addLine()
            telemetry.addLine("<font face=\"monospace\">Xbox/PS4 Button - Motor</font>")
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>")
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>")
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>")
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>")
            telemetry.addLine()

            val hasDriveEncoders = view.leftEncs.isNotEmpty() && view.rightEncs.isNotEmpty()

            if (gamepad1.x) {
                view.leftMotors[0].power = MOTOR_POWER
                telemetry.addLine("Running Motor: Front Left")
                if (hasDriveEncoders) {
                    val pv = view.leftEncs[0].getPositionAndVelocity()
                    telemetry.addLine("Encoder Position: " + pv.position)
                    telemetry.addLine("Encoder Velocity: " + pv.velocity)
                }
            } else if (gamepad1.y) {
                view.rightMotors[0].power = MOTOR_POWER
                telemetry.addLine("Running Motor: Front Right")
                if (hasDriveEncoders) {
                    val pv = view.rightEncs[0].getPositionAndVelocity()
                    telemetry.addLine("Encoder Position: " + pv.position)
                    telemetry.addLine("Encoder Velocity: " + pv.velocity)
                }
            } else if (gamepad1.b) {
                view.rightMotors[1].power = MOTOR_POWER
                telemetry.addLine("Running Motor: Rear Right")
                if (hasDriveEncoders) {
                    val pv = view.rightEncs[1].getPositionAndVelocity()
                    telemetry.addLine("Encoder Position: " + pv.position)
                    telemetry.addLine("Encoder Velocity: " + pv.velocity)
                }
            } else if (gamepad1.a) {
                view.leftMotors[1].power = MOTOR_POWER
                telemetry.addLine("Running Motor: Rear Left")
                if (hasDriveEncoders) {
                    val pv = view.leftEncs[1].getPositionAndVelocity()
                    telemetry.addLine("Encoder Position: " + pv.position)
                    telemetry.addLine("Encoder Velocity: " + pv.velocity)
                }
            } else {
                for (m in view.motors) {
                    m.power = 0.0
                }
                telemetry.addLine("Running Motor: None")
            }

            telemetry.update()
        }
    }
}

class DeadWheelDirectionDebugger(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
        val view = dvf.make(hardwareMap)

        require(view.parEncs.isNotEmpty() && view.perpEncs.isNotEmpty()) {
            "Only run this op mode if you're using dead wheels."
        }

        val telemetry = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)

        waitForStart()

        if (isStopRequested) return

        while (opModeIsActive()) {
            telemetry.addLine("Move each dead wheel individually and make sure the direction is correct")
            telemetry.addLine()

            telemetry.addLine("Parallel Dead Wheels (should increase forward)")
            for (i in view.parEncs.indices) {
                telemetry.addLine("  Wheel $i Position: ${view.parEncs[i].getPositionAndVelocity().position}")
            }
            telemetry.addLine()

            telemetry.addLine("Perpendicular Dead Wheels (should increase leftward)")
            for (i in view.perpEncs.indices) {
                telemetry.addLine("  Wheel $i Position: ${view.perpEncs[i].getPositionAndVelocity().position}")
            }

            telemetry.update()
        }
    }
}
