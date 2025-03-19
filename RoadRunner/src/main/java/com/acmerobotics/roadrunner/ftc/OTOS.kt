package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
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
import kotlin.math.round

fun rawPosVelPair(pos: Int, vel: Int) = PositionVelocityPair(pos, vel, pos, vel)
fun rawPosVelPair(pos: Double, vel: Double) = rawPosVelPair(round(pos).toInt(), round(vel).toInt())

fun SparkFunOTOS.Pose2D.toRRPose() = Pose2d(x, y, h)
fun Pose2d.toOTOSPose() = SparkFunOTOS.Pose2D(position.x, position.y, heading.toDouble())

class OTOSEncoderGroup(val otos: SparkFunOTOS) : EncoderGroup {
    var pos = SparkFunOTOS.Pose2D()
    var vel = SparkFunOTOS.Pose2D()

    override val encoders = listOf(
        ParallelOTOSEncoder(this),
        PerpendicularOTOSEncoder(this)
    )

    override val unwrappedEncoders = encoders

    override fun bulkRead() {
        otos.getPosVelAcc(pos, vel, SparkFunOTOS.Pose2D())
    }
}

class ParallelOTOSEncoder(val group: OTOSEncoderGroup) : Encoder {
    override var direction = DcMotorSimple.Direction.FORWARD

    override fun getPositionAndVelocity() = rawPosVelPair(group.pos.x, group.vel.x)
}

class PerpendicularOTOSEncoder(val group: OTOSEncoderGroup) : Encoder {
    override var direction = DcMotorSimple.Direction.FORWARD

    override fun getPositionAndVelocity() = rawPosVelPair(group.pos.y, group.vel.y)
}

class OTOSIMU(val otos: SparkFunOTOS) : LazyImu, IMU, HardwareDevice by otos {
    override fun initialize(p0: IMU.Parameters?) = otos.initialize()

    override fun resetYaw() = fail()

    override fun getRobotYawPitchRollAngles(): YawPitchRollAngles {
        return YawPitchRollAngles(otos.angularUnit, normalizeAngle(otos.position.h), 0.0, 0.0, 0L)
    }

    override fun getRobotOrientation(
        p0: AxesReference?,
        p1: AxesOrder?,
        p2: AngleUnit?
    ): Orientation = fail()

    override fun getRobotOrientationAsQuaternion(): Quaternion = fail()

    override fun getRobotAngularVelocity(p0: AngleUnit?): AngularVelocity {
        return AngularVelocity(otos.angularUnit, 0.0f, 0.0f, otos.velocity.h.toFloat(), 0L)
    }

    override fun get() = this as IMU

    private fun fail(): Nothing = throw NotImplementedError("Not Needed For Tuning")
}