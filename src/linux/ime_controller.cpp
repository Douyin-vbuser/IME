#include "com_vbuser_ime_IMEController.h"
#include <X11/XKBlib.h>
#include <X11/Xlib.h>

JNIEXPORT void JNICALL Java_com_vbuser_ime_IMEController_toggleIME
(JNIEnv* env, jclass cls, jboolean enable) {
    Display* display = XOpenDisplay(NULL);
    if (display) {
        unsigned int led_mask;
        XkbGetIndicatorState(display, XkbUseCoreKbd, &led_mask);
        bool capsLockState = (led_mask & 1) != 0;
        if (capsLockState != enable) {
            XkbLockModifiers(display, XkbUseCoreKbd, 2, enable ? 2 : 0);
        }
        
        XCloseDisplay(display);
    }
}