We need JNI headers, but not the rest of the JVM when we're cross compiling. Just including jni.h and friends is a reasonable way to deal with this (it's actually what gradle-jni does).

Yes, I know that wpilibsuite/gradle-jni exists. Doing the blessed wpilib way takes more time than I have right now, so we're not using that. TODO
