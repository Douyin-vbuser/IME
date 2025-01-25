#ifndef COM_VBUSER_IME_IMECONTROLLER_H
#define COM_VBUSER_IME_IMECONTROLLER_H

#include <jni.h>

#ifdef __cplusplus
extern "C" {
#endif

    JNIEXPORT void JNICALL Java_com_vbuser_ime_IMEController_toggleIME
    (JNIEnv *, jclass, jboolean);

#ifdef __cplusplus
}
#endif

#endif