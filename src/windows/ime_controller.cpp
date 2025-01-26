#include "com_vbuser_ime_IMEController.h"
#include <windows.h>

JNIEXPORT void JNICALL Java_com_vbuser_ime_IMEController_toggleIME
(JNIEnv* env, jclass cls, jboolean enable) {
    bool capsLockState = (GetKeyState(VK_CAPITAL) & 0x0001) != 0;
    if (capsLockState != enable) {
        keybd_event(VK_CAPITAL, 0x45, KEYEVENTF_EXTENDEDKEY, 0);
        keybd_event(VK_CAPITAL, 0x45, KEYEVENTF_EXTENDEDKEY | KEYEVENTF_KEYUP, 0);
    }
}