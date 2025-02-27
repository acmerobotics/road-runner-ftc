package com.acmerobotics.roadrunner.ftc

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

interface OctoQuadView {
    fun update()

    fun getParEncoderPosition(): Int
    fun getPerpEncoderPosition(): Int
    fun getParEncoderVelocity(): Int
    fun getPerpEncoderVelocity(): Int
    fun getHeadingVelocity(): Float

    fun setParDirection(direction: DcMotorSimple.Direction)
    fun setPerpDirection(direction: DcMotorSimple.Direction)
}

class OctoQuadParEncoder(val octoquad: OctoQuadView) : Encoder {
    override var direction = DcMotorSimple.Direction.FORWARD
        set(value) {
            field = value
            octoquad.setParDirection(value)
        }
    override fun getPositionAndVelocity() = rawPosVelPair(octoquad.getParEncoderPosition(), octoquad.getParEncoderVelocity())
}

class OctoQuadPerpEncoder(val octoquad: OctoQuadView) : Encoder {
    override var direction = DcMotorSimple.Direction.FORWARD
        set(value) {
            field = value
            octoquad.setPerpDirection(value)
        }
    override fun getPositionAndVelocity() = rawPosVelPair(octoquad.getPerpEncoderPosition(), octoquad.getPerpEncoderVelocity())
}

class OctoQuadEncoderGroup(
    val octoquad: OctoQuadView,
) : EncoderGroup {
    override val encoders = listOf(
        OctoQuadParEncoder(octoquad),
        OctoQuadPerpEncoder(octoquad),
    )
    override val unwrappedEncoders = encoders

    override fun bulkRead() {
        octoquad.update()
    }
}

// Only the methods used by tuning routines are implemented
class OctoQuadIMU(val octoquad: OctoQuadView): IMU, LazyImu {
    override fun getManufacturer() = HardwareDevice.Manufacturer.Other
    override fun getDeviceName() = ""
    override fun getConnectionInfo() = ""
    override fun getVersion() = 0

    override fun resetDeviceConfigurationForOpMode() {}

    override fun close() {}

    override fun initialize(parameters: IMU.Parameters?) = true

    override fun resetYaw() {
        throw NotImplementedError()
    }

    override fun getRobotYawPitchRollAngles(): YawPitchRollAngles {
        throw NotImplementedError()
    }

    override fun getRobotOrientation(
        reference: AxesReference?,
        order: AxesOrder?,
        angleUnit: AngleUnit?
    ): Orientation {
        throw NotImplementedError()
    }

    override fun getRobotOrientationAsQuaternion(): Quaternion {
        throw NotImplementedError()
    }

    override fun getRobotAngularVelocity(angleUnit: AngleUnit): AngularVelocity {
        octoquad.update()
        return AngularVelocity(angleUnit, 0.0f, 0.0f,
            octoquad.getHeadingVelocity(), 0L)
    }

    override fun get() = this
}
