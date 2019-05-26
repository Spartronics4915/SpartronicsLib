#include "t265wrapper.hpp"
#include <vector>
#include <fstream>
#include <iterator>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>

// We use jlongs like pointers, so they better be large enough
static_assert(sizeof(jlong) >= sizeof(void *));

// We cache all of these because we can
jclass holdingClass = nullptr; // This should always be T265Camera jclass
jfieldID fieldID = nullptr;    // Field id for the field "nativeCameraObjectPointer"
jclass exception = nullptr;    // This is "CameraJNIException"

// We do this so we don't have to fiddle with files
// Most of the below fields are supposed to be "ignored"
// See https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md#wheel-odometry-calibration-file-format
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

jlong Java_com_spartronics4915_lib_sensors_T265Camera_newCamera(JNIEnv *env, jobject thisObj)
{
    try
    {
        ensureCache(env, thisObj);

        deviceAndSensors *devAndSensors = nullptr;
        try
        {
            devAndSensors = getDeviceFromClass(env, thisObj);
        }
        catch (std::runtime_error)
        {
        }
        if (devAndSensors && devAndSensors->isRunning)
            throw std::runtime_error("Can't make a new camera if the calling class already has one (you need to call free first)");

        auto pipeline = new rs2::pipeline();

        // Imagine writing a C++ wrapper but ignoring most C++ features
        // Looking at you jni.h
        JavaVM *jvm;
        int error = env->GetJavaVM(&jvm);
        if (error)
            throw std::runtime_error("Couldn't get a JavaVM object from the current JNI environment");
        auto globalThis = env->NewGlobalRef(thisObj); // Must be cleaned up later

        /*
         * We define a callback that will run in another thread.
         * This is why we must take care to preserve a pointer to
         * a jvm object, which we attach to the currrent thread
         * so we can get a valid environment object and call
         * callback in Java. We also make a global reference to
         * the current object, which will be cleaned up when the
         * user calls free() from Java.
        */
        auto consumerCallback = [jvm, globalThis](const rs2::frame &frame) {
            JNIEnv *env = nullptr;
            try
            {
                int error = jvm->AttachCurrentThread((void **)&env, nullptr);
                if (error)
                    throw std::runtime_error("Couldn't attach callback thread to jvm");

                auto poseData = frame.as<rs2::pose_frame>().get_pose_data();
                auto q = poseData.rotation;
                // rotation is a quaternion so we must convert to an euler angle (yaw)
                auto yaw = atan2f(2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x));

                auto callbackMethodID = env->GetMethodID(holdingClass, "consumePoseUpdate", "(FFFI)V");
                if (!callbackMethodID)
                    throw std::runtime_error("consumePoseUpdate method doesn't exist");

                env->CallVoidMethod(globalThis, callbackMethodID, poseData.translation.x, poseData.translation.y, yaw, poseData.tracker_confidence);
            }
            catch (std::exception &e)
            {
                /*
                 * Unfortunately if we get an exception while attaching the thread
                 * we can't throw into Java code, so we'll just print to stderr
                */
                if (env)
                    env->ThrowNew(exception, e.what());
                else
                    std::cerr << "Exception in frame consumer callback could not be thrown in Java code: " << e.what() << std::endl;
            }
        };

        auto config = rs2::config();
        config.disable_all_streams();
        config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF); // This will ensure we only get tracking capable devices

        // Start streaming
        auto profile = pipeline->start(config, consumerCallback);

        // Get the currently used device
        auto device = profile.get_device();
        if (!device.is<rs2::tm2>())
        {
            pipeline->stop();
            throw std::runtime_error("The device you have plugged in is not tracking-capable");
        }

        // Get the odometry/pose sensors
        // For the T265 both odom and pose will be from the *same* sensor
        rs2::wheel_odometer *odom = nullptr;
        rs2::pose_sensor *pose = nullptr;
        for (const auto sensor : device.query_sensors())
        {
            if (sensor.is<rs2::wheel_odometer>())
                pose = new rs2::pose_sensor(sensor);
            if (sensor.is<rs2::pose_sensor>())
                odom = new rs2::wheel_odometer(sensor);
        }
        if (!odom)
            throw new std::runtime_error("Selected device does not support wheel odometry inputs");
        if (!pose)
            throw new std::runtime_error("Selected device does not have a pose sensor");

        return reinterpret_cast<jlong>(new deviceAndSensors(pipeline, odom, pose, globalThis));
    }
    catch (std::exception &e)
    {
        ensureCache(env, thisObj);
        env->ThrowNew(exception, e.what());
        return 0;
    }
    return 0;
}

void Java_com_spartronics4915_lib_sensors_T265Camera_sendOdometryRaw(JNIEnv *env, jobject thisObj, jint sensorId, jint frameNumber, jfloat xVel, jfloat yVel)
{
    try
    {
        ensureCache(env, thisObj);

        auto devAndSensors = getDeviceFromClass(env, thisObj);
        // jints are 32 bit and are signed so we have to be careful
        // jint shouldn't be able to be greater than UINT32_MAX, but we'll be defensive
        if (sensorId > UINT8_MAX || frameNumber > UINT32_MAX || sensorId < 0 || frameNumber < 0)
            env->ThrowNew(exception, "sensorId or frameNumber are out of range");

        devAndSensors->wheelOdometrySensor->send_wheel_odometry(sensorId, frameNumber, rs2_vector{.x = xVel, .y = yVel, .z = 0.0});
    }
    catch (std::exception &e)
    {
        env->ThrowNew(exception, e.what());
    }
}

void Java_com_spartronics4915_lib_sensors_T265Camera_exportRelocalizationMap(JNIEnv *env, jobject thisObj, jstring savePath)
{
    try
    {
        ensureCache(env, thisObj);

        auto pathNativeStr = env->GetStringUTFChars(savePath, 0);

        // Open file in binary mode
        auto file = std::ofstream(pathNativeStr, std::ios::binary);
        if (!file || file.bad())
            throw std::runtime_error("Couldn't open file to write a relocalization map");

        // Get data from sensor and write
        auto devAndSensors = getDeviceFromClass(env, thisObj);
        devAndSensors->pipeline->stop();
        devAndSensors->isRunning = false;

        // I know, this is really gross...
        // Unfortunately there is apparently no way to figure out if we're ready to export the map
        std::this_thread::sleep_for(std::chrono::seconds(1));

        auto data = devAndSensors->poseSensor->export_localization_map();
        file.write(reinterpret_cast<const char *>(data.begin().base()), data.size());

        env->ReleaseStringUTFChars(savePath, pathNativeStr);
        file.close();

        // TODO: Camera never gets started again...
        // If we try to call pipeline->start() it doesn't work. Bug in librealsense?
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

        // Open file and make a vector to hold contents
        auto file = std::ifstream(pathNativeStr, std::ios::binary);
        if (!file || file.bad())
            throw std::runtime_error("Couldn't open file to read a relocalization map");

        auto dataVec = std::vector<uint8_t>(
            std::istreambuf_iterator<char>(file),
            std::istreambuf_iterator<char>());

        // Pass contents to the pose sensor
        auto devAndSensors = getDeviceFromClass(env, thisObj);
        auto success = devAndSensors->poseSensor->import_localization_map(dataVec);
        if (!success)
            throw std::runtime_error("import_localization_map returned a value indicating failure");

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
        auto vecBuf = std::vector<uint8_t>(*buf, *buf + size);

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
    try
    {
        ensureCache(env, thisObj);

        auto devAndSensors = getDeviceFromClass(env, thisObj);
        if (devAndSensors->isRunning)
            devAndSensors->pipeline->stop();
        env->DeleteGlobalRef(devAndSensors->globalThis);

        delete devAndSensors;

        env->SetLongField(thisObj, fieldID, 0);
    }
    catch (std::exception &e)
    {
        env->ThrowNew(exception, e.what());
    }
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
    {
        auto lHoldingClass = env->GetObjectClass(thisObj);
        holdingClass = reinterpret_cast<jclass>(env->NewGlobalRef(lHoldingClass));
    }
    if (!fieldID)
    {
        fieldID = env->GetFieldID(holdingClass, "mNativeCameraObjectPointer", "J");
    }
    if (!exception)
    {
        auto lException = env->FindClass("com/spartronics4915/lib/sensors/T265Camera$CameraJNIException");
        exception = reinterpret_cast<jclass>(env->NewGlobalRef(lException));
    }
}