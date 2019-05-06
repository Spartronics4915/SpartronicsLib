#include "t265wrapper.hpp"
#include <librealsense2/rs.hpp> 

// We use jlongs like pointers, so they better be large enough
static_assert(sizeof(jlong) >= sizeof(void *));

// We cache all of these because we can
jclass holdingClass = nullptr; // This should always be T265Camera
jfieldID fieldID = nullptr; // Field id for the field "nativeCameraObjectPointer"
jclass exception = nullptr; // This is "CameraJNIException"

auto const context = new rs2::context();

class deviceAndSensors
{
    public:

    deviceAndSensors(rs2::tm2 *dev, rs2::wheel_odometer *odom, rs2::pose_sensor *pose) : device(dev), wheelOdometrySensor(odom), poseSensor(pose)
    {}

    ~deviceAndSensors()
    {
        delete device;
        delete wheelOdometrySensor;
        delete poseSensor;
    }

    const rs2::tm2 *device;
    const rs2::wheel_odometer *wheelOdometrySensor;
    const rs2::pose_sensor *poseSensor;
};

jlong Java_com_spartronics4915_lib_sensors_T265Camera_newCamera(JNIEnv *env, jobject thisObj)
{
    ensureCache(env, thisObj);
    try
    {
        auto devices = context->query_devices();
        if (devices.size() <= 0)
            env->ThrowNew(exception, "No RealSense devices found... Are they unplugged?");
        // For now just get the first one
        auto device = new rs2::tm2(devices[0]);
        
        rs2::wheel_odometer *odom = nullptr;
        rs2::pose_sensor *pose = nullptr;
        for (const auto sensor: device->query_sensors())
        {
            if (rs2_is_sensor_extendable_to(sensor.get().get(), RS2_EXTENSION_POSE, nullptr))
                pose = new rs2::pose_sensor(sensor);
            if (rs2_is_sensor_extendable_to(sensor.get().get(), RS2_EXTENSION_WHEEL_ODOMETER, nullptr))
                odom = new rs2::wheel_odometer(sensor);
        }
        if (!odom)
            env->ThrowNew(exception, "Selected device does not support returning wheel odometry");
        if (!pose)
            env->ThrowNew(exception, "Selected device does not have a pose sensor");

        return reinterpret_cast<jlong>(new deviceAndSensors(device, odom, pose));
    }
    catch (std::exception& e)
    {
        env->ThrowNew(exception, e.what());
        return static_cast<jlong>(0);
    }
}

void Java_com_spartronics4915_lib_sensors_T265Camera_stopCamera(JNIEnv *env, jobject thisObj)
{
    auto devAndSensors = getDeviceFromClass(env, thisObj);
    devAndSensors->poseSensor->stop();
    devAndSensors->wheelOdometrySensor->stop();
}

void Java_com_spartronics4915_lib_sensors_T265Camera_sendOdometryRaw(JNIEnv *env, jobject thisObj, jint sensorId, jint frameNumber, jfloat xVel, jfloat yVel)
{
    auto devAndSensors = getDeviceFromClass(env, thisObj);
    if (sensorId > UINT8_MAX || frameNumber > UINT32_MAX || sensorId < 0 || frameNumber < 0)
        env->ThrowNew(exception, "sensorId or frameNumber are out of range");
    // TODO
}

void Java_com_spartronics4915_lib_sensors_T265Camera_openAndStartCamera(JNIEnv *env, jobject thisObj, jobject frameRecvCallback)
{
    // TODO: Remove
}

void Java_com_spartronics4915_lib_sensors_T265Camera_loadRelocalizationMap(JNIEnv *env, jobject thisObj, jstring mapPath)
{
    // TODO
}

void Java_com_spartronics4915_lib_sensors_T265Camera_setOdometryInfo(JNIEnv *env, jobject thisObj, jfloat xOffset, jfloat yOffset, jfloat angOffset, jfloat measureCovariance)
{
    // TODO
}

void Java_com_spartronics4915_lib_sensors_T265Camera_free(JNIEnv *env, jobject thisObj)
{
    delete getDeviceFromClass(env, thisObj);
}

deviceAndSensors *getDeviceFromClass(JNIEnv *env, jobject thisObj)
{
    ensureCache(env, thisObj);

    auto pointer = env->GetLongField(thisObj, fieldID);
    if (pointer == 0)
        env->ThrowNew(exception, "nativeCameraObjectPointer cannot be 0");
    return reinterpret_cast<deviceAndSensors *>(pointer);
}

void ensureCache(JNIEnv *env, jobject thisObj)
{
    if (!holdingClass)
        holdingClass = env->GetObjectClass(thisObj);
    if (!fieldID)
        fieldID = env->GetFieldID(holdingClass, "nativeCameraObjectPointer", "J");
    if (!exception)
        exception = env->FindClass("com/spartronics4915/lib/sensors/CameraJNIException");
}