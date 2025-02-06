package com.acmerobotics.roadrunner.ftc

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D
import com.qualcomm.robotcore.hardware.DcMotorSimple

enum class EncoderDirection {
    PARALLEL,
    PERPENDICULAR
}

class OTOSEncoder(val otos: SparkFunOTOS, val ed: EncoderDirection) : Encoder {
    override var direction = DcMotorSimple.Direction.FORWARD

    override fun getPositionAndVelocity(): PositionVelocityPair {
        val otosPose: Pose2D = Pose2D()
        val otosVel: Pose2D = Pose2D()
        otos.getPosVelAcc(otosPose, otosVel, Pose2D())

        return when (ed) {
            EncoderDirection.PARALLEL -> PositionVelocityPair(otosPose.x.toInt(), otosVel.x.toInt(), otosPose.x.toInt(), otosVel.x.toInt())
            EncoderDirection.PERPENDICULAR -> PositionVelocityPair(otosPose.y.toInt(), otosVel.y.toInt(), otosPose.y.toInt(), otosVel.y.toInt())
        }
    }

}