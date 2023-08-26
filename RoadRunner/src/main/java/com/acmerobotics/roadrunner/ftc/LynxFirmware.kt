@file:JvmName("LynxFirmware")

package com.acmerobotics.roadrunner.ftc

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Parsed representation of a Lynx module firmware version.
 */
class LynxFirmwareVersion(
        val major: Int,
        val minor: Int,
        val eng: Int
) {
    operator fun compareTo(other: LynxFirmwareVersion): Int {
        val majorComp = major.compareTo(other.major)
        return if (majorComp == 0) {
            val minorComp = minor.compareTo(other.minor)
            if (minorComp == 0) {
                eng.compareTo(other.eng)
            } else {
                minorComp
            }
        } else {
            majorComp
        }
    }

    override fun toString() = "$major.$minor.$eng"
}

/**
 * Parses Lynx module firmware version string into structured version object. Returns null for
 * null argument or upon error.
 */
fun parse(s: String?): LynxFirmwareVersion? {
    if (s == null) {
        return null
    }

    val parts = s.split("[ :,]+".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray()
    return try {
        // For now, we ignore the hardware entry
        LynxFirmwareVersion(
                Integer.parseInt(parts[3]),
                Integer.parseInt(parts[5]),
                Integer.parseInt(parts[7])
        )
    } catch (e: NumberFormatException) {
        null
    }
}

private val MIN_VERSION = LynxFirmwareVersion(1, 8, 2)

/**
 * Ensure all of the Lynx modules attached to the robot satisfy the minimum requirement.
 * @param hardwareMap hardware map containing Lynx modules
 */
fun throwIfModulesAreOutdated(hardwareMap: HardwareMap) {
    val msgBuilder = StringBuilder()

    for (module in hardwareMap.getAll(LynxModule::class.java)) {
        val version = parse(module.nullableFirmwareVersionString)
        if (version == null || version < MIN_VERSION) {
            for (name in hardwareMap.getNamesOf(module)) {
                msgBuilder.append("\t$name: ${version ?: "(unparseable) ${module.nullableFirmwareVersionString}"}\n")
            }
        }
    }

    if (msgBuilder.isNotEmpty()) {
        msgBuilder.insert(0, "Mandatory minimum firmware version for Road Runner: $MIN_VERSION\n")
        msgBuilder.insert(0, "One or more of the attached Lynx modules has outdated firmware\n")
        throw RuntimeException(msgBuilder.toString())
    }
}
