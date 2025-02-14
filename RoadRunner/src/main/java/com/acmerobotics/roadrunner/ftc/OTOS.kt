package com.acmerobotics.roadrunner.ftc

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D
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

enum class EncoderDirection {
    PARALLEL,
    PERPENDICULAR
}

class OTOSView(val otos: SparkFunOTOS) {
    var pose = Pose2D()
        private set

    var vel = Pose2D()
        private set

    fun update() = otos.getPosVelAcc(pose, vel, Pose2D())
}

class OTOSEncoder(val view: OTOSView, val ed: EncoderDirection) : Encoder {
    override var direction = DcMotorSimple.Direction.FORWARD

    override fun getPositionAndVelocity(): PositionVelocityPair {
        return when (ed) {
            EncoderDirection.PARALLEL -> PositionVelocityPair(view.pose.x.toInt(), view.vel.x.toInt(), view.pose.x.toInt(), view.vel.x.toInt())
            EncoderDirection.PERPENDICULAR -> PositionVelocityPair(view.pose.y.toInt(), view.vel.y.toInt(), view.pose.y.toInt(), view.vel.y.toInt())
        }
    }
}

class OTOSEncoderGroup(val view: OTOSView) : EncoderGroup {
    override val encoders = listOf(
        OTOSEncoder(view, EncoderDirection.PARALLEL),
        OTOSEncoder(view, EncoderDirection.PERPENDICULAR),
    )

    override val unwrappedEncoders = encoders

    override fun bulkRead() = view.update()
}

class OTOSIMU(val view: OTOSView) : LazyImu, IMU, HardwareDevice by view.otos {
    override fun initialize(p0: IMU.Parameters?) = view.otos.initialize()

    override fun resetYaw() = fail()

    override fun getRobotYawPitchRollAngles(): YawPitchRollAngles {
        return YawPitchRollAngles(AngleUnit.RADIANS, view.pose.h, 0.0, 0.0, 0L)
    }

    override fun getRobotOrientation(
        p0: AxesReference?,
        p1: AxesOrder?,
        p2: AngleUnit?
    ): Orientation = fail()

    override fun getRobotOrientationAsQuaternion(): Quaternion = fail()

    override fun getRobotAngularVelocity(p0: AngleUnit?): AngularVelocity {
        return AngularVelocity(AngleUnit.RADIANS, 0.0f, 0.0f, view.vel.h.toFloat(), 0L)
    }

    override fun get() = this as IMU

    private fun fail(): Nothing = throw NotImplementedError("Not Needed For Tuning")
}