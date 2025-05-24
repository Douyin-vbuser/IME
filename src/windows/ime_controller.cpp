#include "com_vbuser_ime_IMEController.h"
#include <windows.h>
#include <Imm.h>
#pragma comment(lib, "Imm32.lib")

JNIEXPORT void JNICALL Java_com_vbuser_ime_IMEController_toggleIME(
    JNIEnv* env, jclass cls, jboolean enable) {

    HIMC hIMC = ImmGetContext(GetForegroundWindow());
    if (hIMC == nullptr) {
        return;
    }

    DWORD dwConversion;
    DWORD dwSentence;

    if (ImmGetConversionStatus(hIMC, &dwConversion, &dwSentence)) {
        if (enable) {
            dwConversion |= IME_CMODE_NATIVE;
        } else {
            dwConversion &= ~IME_CMODE_NATIVE;
        }
        ImmSetConversionStatus(hIMC, dwConversion, dwSentence);
    }

    ImmReleaseContext(GetForegroundWindow(), hIMC);
}