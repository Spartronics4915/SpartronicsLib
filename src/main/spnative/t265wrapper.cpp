#include "t265wrapper.hpp"
#include <vector>
#include <fstream>
#include <iterator>

// We use jlongs like pointers, so they better be large enough
static_assert(sizeof(jlong) >= sizeof(void *));

// We cache all of these because we can
jclass holdingClass = nullptr; // This should always be T265Camera jclass
jfieldID fieldID = nullptr;    // Field id for the field "nativeCameraObjectPointer"
jclass exception = nullptr;    // This is "CameraJNIException"

rs2::context *context = nullptr;

auto const odometryConfig = R"(
{
    "velocimeters": [
        {
            "scale_and_alignment": [
                1.0,
                0.0000000000000000,
                0.0000000000000000,
                0.0000000000000000,
                1.0,
                0.0000000000000000,
                0.0000000000000000,
                0.0000000000000000,
                1.0
            ],
            "noise_variance": %f,
            "extrinsics": {
                "T": [
                    %f,
                    %f,
                    0.0
                ],
                "T_variance": [
                    9.999999974752427e-7, 
                    9.999999974752427e-7, 
                    9.999999974752427e-7
                ],
                "W": [
                    0.0,
                    %f,
                    0.0
                ],
                "W_variance": [
                    9.999999974752427e-5, 
                    9.999999974752427e-5, 
                    9.999999974752427e-5
                ]
            }
        }
    ]
}
)";
// We do this so we don't have to fiddle with files
// Most of the above fields are supposed to be "ignored"
// See https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md#wheel-odometry-calibration-file-format

jlong Java_com_spartronics4915_lib_sensors_T265Camera_newCamera(JNIEnv *env, jobject thisObj, jobject supplier)
{
    try
    {
        if (!context)
            context = new rs2::context();
        
        auto devices = context->query_devices();
        if (devices.size() <= 0)
            throw new std::runtime_error("No RealSense device found... Is it unplugged?");
        // For now just get the first device
        auto device = new rs2::tm2(devices[0]);

        rs2::wheel_odometer *odom = nullptr;
        rs2::pose_sensor *pose = nullptr;
        for (const auto sensor : device->query_sensors())
        {
            if (rs2_is_sensor_extendable_to(sensor.get().get(), RS2_EXTENSION_POSE, nullptr))
                pose = new rs2::pose_sensor(sensor);
            if (rs2_is_sensor_extendable_to(sensor.get().get(), RS2_EXTENSION_WHEEL_ODOMETER, nullptr))
                odom = new rs2::wheel_odometer(sensor);
        }
        if (!odom)
            throw new std::runtime_error("Selected device does not support returning wheel odometry");
        if (!pose)
            throw new std::runtime_error("Selected device does not have a pose sensor");

        // TODO: Set callback on poseSensor

        return reinterpret_cast<jlong>(new deviceAndSensors(device, odom, pose));
    }
    catch (std::exception &e)
    {
        puts(e.what());
        ensureCache(env, thisObj);
        env->ThrowNew(exception, e.what());
        return static_cast<jlong>(0);
    }
}

void Java_com_spartronics4915_lib_sensors_T265Camera_startCamera(JNIEnv *env, jobject thisObj)
{
    try
    {
        ensureCache(env, thisObj);

        auto devAndSensors = getDeviceFromClass(env, thisObj);
        // TODO: Set callback and start
        // devAndSensors->poseSensor->start();
        // devAndSensors->wheelOdometrySensor->start();
    }
    catch (std::exception &e)
    {
        env->ThrowNew(exception, e.what());
    }
}

void Java_com_spartronics4915_lib_sensors_T265Camera_stopCamera(JNIEnv *env, jobject thisObj)
{
    try
    {
        ensureCache(env, thisObj);

        auto devAndSensors = getDeviceFromClass(env, thisObj);
        devAndSensors->poseSensor->stop();
        devAndSensors->wheelOdometrySensor->stop();
    }
    catch (std::exception &e)
    {
        env->ThrowNew(exception, e.what());
    }
}

void Java_com_spartronics4915_lib_sensors_T265Camera_sendOdometryRaw(JNIEnv *env, jobject thisObj, jint sensorId, jint frameNumber, jfloat xVel, jfloat yVel)
{
    try
    {
        ensureCache(env, thisObj);

        auto devAndSensors = getDeviceFromClass(env, thisObj);
        if (sensorId > UINT8_MAX || frameNumber > UINT32_MAX || sensorId < 0 || frameNumber < 0)
            env->ThrowNew(exception, "sensorId or frameNumber are out of range");

        devAndSensors->wheelOdometrySensor->send_wheel_odometry(sensorId, frameNumber, rs2_vector{.x = xVel, .y = yVel, .z = 0.0});
    }
    catch (std::exception &e)
    {
        env->ThrowNew(exception, e.what());
    }
}

void Java_com_spartronics4915_lib_sensors_T265Camera_loadRelocalizationMap(JNIEnv *env, jobject thisObj, jstring mapPath)
{
    try
    {
        ensureCache(env, thisObj);

        auto pathNativeStr = env->GetStringUTFChars(mapPath, 0);

        // Open file and get info
        auto file = std::basic_ifstream<uint8_t>(pathNativeStr, std::ios::binary);
        auto dataVec = std::vector<uint8_t>(
            std::istreambuf_iterator<uint8_t>(file),
            std::istreambuf_iterator<uint8_t>());

        // Pass contents to the pose sensor
        auto devAndSensors = getDeviceFromClass(env, thisObj);
        devAndSensors->poseSensor->import_localization_map(dataVec);

        env->ReleaseStringUTFChars(mapPath, pathNativeStr);
        file.close();
    }
    catch (std::exception &e)
    {
        env->ThrowNew(exception, e.what());
    }
}

void Java_com_spartronics4915_lib_sensors_T265Camera_setOdometryInfo(JNIEnv *env, jobject thisObj, jfloat xOffset, jfloat yOffset, jfloat angOffset, jfloat measureCovariance)
{
    try
    {
        ensureCache(env, thisObj);

        auto size = snprintf(nullptr, 0, odometryConfig, measureCovariance, xOffset, yOffset, angOffset);
        char buf[size];
        snprintf(buf, size, odometryConfig, xOffset, yOffset, angOffset);
        auto vecBuf = std::vector<uint8_t>(*buf, *buf+size);

        auto devAndSensors = getDeviceFromClass(env, thisObj);
        devAndSensors->wheelOdometrySensor->load_wheel_odometery_config(vecBuf);
    }
    catch (std::exception &e)
    {
        env->ThrowNew(exception, e.what());
    }
}

void Java_com_spartronics4915_lib_sensors_T265Camera_free(JNIEnv *env, jobject thisObj)
{
    delete getDeviceFromClass(env, thisObj);
}

deviceAndSensors *getDeviceFromClass(JNIEnv *env, jobject thisObj)
{
    auto pointer = env->GetLongField(thisObj, fieldID);
    if (pointer == 0)
        throw std::runtime_error("nativeCameraObjectPointer cannot be 0");
    return reinterpret_cast<deviceAndSensors *>(pointer);
}

void ensureCache(JNIEnv *env, jobject thisObj)
{
    if (!holdingClass)
        holdingClass = env->GetObjectClass(thisObj);
    if (!fieldID)
        fieldID = env->GetFieldID(holdingClass, "nativeCameraObjectPointer", "J");
    if (!exception)
        exception = env->FindClass("com/spartronics4915/lib/sensors/T265Camera$CameraJNIException");
}