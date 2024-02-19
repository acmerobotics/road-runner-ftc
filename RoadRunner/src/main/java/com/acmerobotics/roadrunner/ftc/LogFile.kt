@file:JvmName("LogFile")

package com.acmerobotics.roadrunner.ftc

import java.io.OutputStream
import java.lang.reflect.Modifier
import java.nio.ByteBuffer
import java.util.TreeMap

sealed interface MessageSchema {
    // schema encoding
    fun encodedSize(): Int  // in bytes
    fun encode(b: ByteBuffer)

    // object encoding
    fun encodedObjSize(o: Any): Int  // in bytes
    fun encodeObj(o: Any, b: ByteBuffer)
}

class StructSchema(
        val fields: Map<String, MessageSchema>,
) : MessageSchema {
    override fun encodedSize() = 8 + fields.map { (name, schema) ->
        4 + name.toByteArray(Charsets.UTF_8).size + schema.encodedSize()
    }.sum()

    override fun encode(b: ByteBuffer) {
        b.putInt(0)  // tag
        b.putInt(fields.size)
        for ((name, schema) in fields) {
            val bytes = name.toByteArray(Charsets.UTF_8)
            b.putInt(bytes.size)
            b.put(bytes)
            schema.encode(b)
        }
    }

    override fun encodedObjSize(o: Any) =
            fields.map { (name, schema) ->
                val field = o.javaClass.getField(name)
                schema.encodedObjSize(field.get(o)!!)
            }.sum()

    override fun encodeObj(o: Any, b: ByteBuffer) {
        for ((name, schema) in fields) {
            val field = o.javaClass.getField(name)
            schema.encodeObj(field.get(o)!!, b)
        }
    }
}

enum class PrimitiveSchema : MessageSchema {
    INT {
        override fun encodedSize() = 4
        override fun encode(b: ByteBuffer) { b.putInt(1) }

        override fun encodedObjSize(o: Any) = 4
        override fun encodeObj(o: Any, b: ByteBuffer) { b.putInt(o as Int) }
    },
    LONG {
        override fun encodedSize() = 4
        override fun encode(b: ByteBuffer) { b.putInt(2) }

        override fun encodedObjSize(o: Any) = 8
        override fun encodeObj(o: Any, b: ByteBuffer) { b.putLong(o as Long) }
    },
    DOUBLE {
        override fun encodedSize() = 4
        override fun encode(b: ByteBuffer) { b.putInt(3) }

        override fun encodedObjSize(o: Any) = 8
        override fun encodeObj(o: Any, b: ByteBuffer) { b.putDouble(o as Double) }
    },
    STRING {
        override fun encodedSize() = 4
        override fun encode(b: ByteBuffer) { b.putInt(4) }

        override fun encodedObjSize(o: Any) = 4 + (o as String).toByteArray(Charsets.UTF_8).size
        override fun encodeObj(o: Any, b: ByteBuffer) {
            val bytes = (o as String).toByteArray(Charsets.UTF_8)
            b.putInt(bytes.size)
            b.put(bytes)
        }
    },
    BOOLEAN {
        override fun encodedSize() = 4
        override fun encode(b: ByteBuffer) { b.putInt(5) }

        override fun encodedObjSize(o: Any) = 1
        override fun encodeObj(o: Any, b: ByteBuffer) { b.put(when (o as Boolean) { true -> 1; false -> 0 }.toByte()) }
    };
}

// TODO: consider replacing enumClass with a list of enum constants
class EnumSchema(val enumClass: Class<Enum<*>>) : MessageSchema {
    init {
        require(enumClass.isEnum)
    }

    override fun encodedSize() = 8 + enumClass.enumConstants!!.sumOf { constant ->
        4 + constant.name.toByteArray(Charsets.UTF_8).size
    }

    override fun encode(b: ByteBuffer) {
        b.putInt(6) // tag
        val constants = enumClass.enumConstants!!
        b.putInt(constants.size)
        for (constant in constants) {
            val bytes = constant.name.toByteArray(Charsets.UTF_8)
            b.putInt(bytes.size)
            b.put(bytes)
        }
    }


    override fun encodedObjSize(o: Any) = 4

    override fun encodeObj(o: Any, b: ByteBuffer) {
        val constant = o as Enum<*>
        b.putInt(constant.ordinal)
    }
}

class ArraySchema(val schema: MessageSchema) : MessageSchema {
    override fun encodedSize() = 8 + schema.encodedSize()

    override fun encode(b: ByteBuffer) {
        b.putInt(7) // tag
        schema.encode(b)
    }

    fun cast(o: Any): Array<*> {
        return when (o) {
            is IntArray -> o.toTypedArray()
            is LongArray -> o.toTypedArray()
            is DoubleArray -> o.toTypedArray()
            is BooleanArray -> o.toTypedArray()
            is Array<*> -> o
            else -> throw IllegalArgumentException("unsupported array type: ${o.javaClass}")
        }
    }

    override fun encodedObjSize(o: Any) = 4 + cast(o).sumOf { schema.encodedObjSize(it!!) }

    override fun encodeObj(o: Any, b: ByteBuffer) {
        val a = cast(o)
        b.putInt(a.size)
        for (e in a) {
            schema.encodeObj(e!!, b)
        }
    }
}

fun schemaOfClass(c: Class<*>): MessageSchema =
        when (c) {
            Int::class.java, Int::class.javaObjectType -> PrimitiveSchema.INT
            Long::class.java, Long::class.javaObjectType -> PrimitiveSchema.LONG
            Double::class.java, Double::class.javaObjectType -> PrimitiveSchema.DOUBLE
            String::class.java, String::class.javaObjectType -> PrimitiveSchema.STRING
            Boolean::class.java, Boolean::class.javaObjectType -> PrimitiveSchema.BOOLEAN
            else -> {
                if (c.isArray) {
                    ArraySchema(schemaOfClass(c.componentType!!))
                } else if (c.isEnum) {
                    // TODO: is there a way to make this cast safe?
                    EnumSchema(c as Class<Enum<*>>)
                } else {
                    val fields = TreeMap<String, MessageSchema>()
                    for (f in c.fields) {
                        require(!Modifier.isStatic(f.modifiers)) { "static fields not supported" }
                        fields[f.name] = schemaOfClass(f.type)
                    }

                    require(fields.isNotEmpty()) { "empty structs not supported" }

                    StructSchema(fields)
                }
            }
        }

class LogWriter(val os: OutputStream) {
    init {
        os.write(ByteBuffer.allocate(4).put('R'.code.toByte()).put('R'.code.toByte()).putShort(0).array()) // magic + version
    }

    data class ChannelMetadata(
            val className: String,
            val index: Int,
            val schema: MessageSchema,
    )

    private val metadata = mutableMapOf<String, ChannelMetadata>()
    private var nextIndex = 0

    fun write(ch: String, o: Any) {
        val mLast = metadata[ch]
        val m = if (mLast == null) {
            val schema = schemaOfClass(o.javaClass)

            val m = ChannelMetadata(o.javaClass.name, nextIndex, schema)
            metadata[ch] = m
            nextIndex++

            val chBytes = ch.toByteArray(Charsets.UTF_8)
            val b = ByteBuffer.allocate(8 + chBytes.size + schema.encodedSize())
            b.putInt(0) // schema entry
            b.putInt(chBytes.size)
            b.put(chBytes)
            schema.encode(b)

            os.write(b.array())

            m
        } else {
            require(mLast.className == o.javaClass.name)

            mLast
        }

        val b = ByteBuffer.allocate(8 + m.schema.encodedObjSize(o))
        b.putInt(1) // message entry
        b.putInt(m.index) // channel index
        m.schema.encodeObj(o, b)
        os.write(b.array())
    }
}
