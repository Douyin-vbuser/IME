#include "com_vbuser_ime_IMEController.h"
#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <X11/XKBlib.h>

void SetEnglishKeyboardLayout(Display* display) {
    XkbStateRec xkbState;
    Status result = XkbGetState(display, XkbUseCoreKbd, &xkbState);
    if (result == Success) {
        Bool lockResult = XkbLockGroup(display, XkbUseCoreKbd, 0);
        if (!lockResult) {
        }
    } else {
    }
}

JNIEXPORT void JNICALL Java_com_vbuser_ime_IMEController_toggleIME
  (JNIEnv* env, jclass cls, jboolean enable) {
    Display* display = XOpenDisplay(NULL);
    if (display != NULL) {
        if (enable) {
            SetEnglishKeyboardLayout(display);
        }
        XCloseDisplay(display);
    }
}