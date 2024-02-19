package com.acmerobotics.roadrunner.ftc

import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows
import java.io.ByteArrayOutputStream

// companion objects aren't allowed in local classes for some reason
class SingleStaticMember(
        @JvmField
        var x: Int = 0
) {
    companion object {
        @JvmStatic
        var y = 10
    }
}

object LogWriterTest {
    @Test
    fun testSingleStaticMember() {
        assertThrows<IllegalArgumentException> {
            ByteArrayOutputStream().use {
                val writer = LogWriter(it)
                writer.write("TEST_CHANNEL", SingleStaticMember())
            }
        }
    }

    @Test
    fun testEmptyClass() {
        class Empty

        assertThrows<IllegalArgumentException> {
            ByteArrayOutputStream().use {
                val writer = LogWriter(it)
                writer.write("TEST_CHANNEL", Empty())
            }
        }
    }

    @Test
    fun testArrayMembers() {
        class ArrayMembers(
                @JvmField
                var x: IntArray,
                @JvmField
                var y: Array<Int>,
                @JvmField
                var z: Array<Array<Int>>,
                @JvmField
                var w: Array<Array<Array<Int>>>,
        )

        ByteArrayOutputStream().use {
            val writer = LogWriter(it)
            writer.write("TEST_CHANNEL", ArrayMembers(
                    intArrayOf(1, 2, 3),
                    arrayOf(1, 2, 3),
                    arrayOf(arrayOf(1, 2, 3)),
                    arrayOf(arrayOf(arrayOf(1, 2, 3)))
            ))
        }
    }

    @Test
    fun testEncoding() {
        class Nested(
                @JvmField
                var m: PrimitiveSchema,
        )

        class Data(
                @JvmField
                var w: Boolean,
                @JvmField
                var x: Int,
                @JvmField
                var y: Long,
                @JvmField
                var z: Double,
                @JvmField
                var s: String,
                @JvmField
                var n: Nested
        )
        val data = Data(true, 1, 2, 3.0, "hello",
            Nested(PrimitiveSchema.DOUBLE)
        )

        val baos = ByteArrayOutputStream()
        baos.use {
            val writer = LogWriter(it)
            writer.write("TEST_CHANNEL", data)
            writer.write("TEST_CHANNEL", data)
            writer.write("TEST_CHANNEL", data)
            writer.write("TEST_CHANNEL2", data.n)
        }

        val contents = baos.toByteArray()
        val expectedContents = byteArrayOf(82, 82, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 84, 69, 83, 84, 95, 67, 72, 65, 78, 78, 69, 76, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 1, 110, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 109, 0, 0, 0, 6, 0, 0, 0, 5, 0, 0, 0, 3, 73, 78, 84, 0, 0, 0, 4, 76, 79, 78, 71, 0, 0, 0, 6, 68, 79, 85, 66, 76, 69, 0, 0, 0, 6, 83, 84, 82, 73, 78, 71, 0, 0, 0, 7, 66, 79, 79, 76, 69, 65, 78, 0, 0, 0, 1, 115, 0, 0, 0, 4, 0, 0, 0, 1, 119, 0, 0, 0, 5, 0, 0, 0, 1, 120, 0, 0, 0, 1, 0, 0, 0, 1, 121, 0, 0, 0, 2, 0, 0, 0, 1, 122, 0, 0, 0, 3, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 5, 104, 101, 108, 108, 111, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 64, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 5, 104, 101, 108, 108, 111, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 64, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 5, 104, 101, 108, 108, 111, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 64, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 84, 69, 83, 84, 95, 67, 72, 65, 78, 78, 69, 76, 50, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 109, 0, 0, 0, 6, 0, 0, 0, 5, 0, 0, 0, 3, 73, 78, 84, 0, 0, 0, 4, 76, 79, 78, 71, 0, 0, 0, 6, 68, 79, 85, 66, 76, 69, 0, 0, 0, 6, 83, 84, 82, 73, 78, 71, 0, 0, 0, 7, 66, 79, 79, 76, 69, 65, 78, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 2)
        assertArrayEquals(expectedContents, contents)
    }

    @Test
    fun testEncodingPrimitive() {
        ByteArrayOutputStream().use {
            val writer = LogWriter(it)
            writer.write("TEST_CHANNEL", 1L)
        }
    }
}
