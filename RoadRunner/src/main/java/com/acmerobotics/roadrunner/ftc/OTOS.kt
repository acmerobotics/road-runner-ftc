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

class OTOSEncoder(val otos: SparkFunOTOS, val ed: EncoderDirection) : Encoder {
    override var direction = DcMotorSimple.Direction.FORWARD

    override fun getPositionAndVelocity(): PositionVelocityPair {
        val otosPose = Pose2D()
        val otosVel = Pose2D()
        otos.getPosVelAcc(otosPose, otosVel, Pose2D())

        return when (ed) {
            EncoderDirection.PARALLEL -> PositionVelocityPair(otosPose.x.toInt(), otosVel.x.toInt(), otosPose.x.toInt(), otosVel.x.toInt())
            EncoderDirection.PERPENDICULAR -> PositionVelocityPair(otosPose.y.toInt(), otosVel.y.toInt(), otosPose.y.toInt(), otosVel.y.toInt())
        }
    }
}

class OTOSIMU(val otos: SparkFunOTOS) : IMU, HardwareDevice by otos {
    override fun initialize(p0: IMU.Parameters?) = otos.initialize()

    override fun resetYaw() {
        otos.calibrateImu()
    }

    override fun getRobotYawPitchRollAngles(): YawPitchRollAngles {
        return YawPitchRollAngles(AngleUnit.RADIANS, otos.position.h, 0.0, 0.0, 0L)
    }

    override fun getRobotOrientation(
        p0: AxesReference?,
        p1: AxesOrder?,
        p2: AngleUnit?
    ): Orientation {
       throw NotImplementedError()
    }

    override fun getRobotOrientationAsQuaternion(): Quaternion {
        TODO("Not yet implemented")
    }

    override fun getRobotAngularVelocity(p0: AngleUnit?): AngularVelocity {
        return AngularVelocity(AngleUnit.RADIANS, 0.0f, 0.0f, otos.velocity.h.toFloat(), 0L)
    }

}