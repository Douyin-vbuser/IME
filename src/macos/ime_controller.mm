#include "com_vbuser_ime_IMEController.h"
#import <Foundation/Foundation.h>
#import <Carbon/Carbon.h>

bool getCapsLockState() {
    CFTypeRef capsLockState = TISGetInputSourceProperty(TISCopyCurrentKeyboardInputSource(), kTISPropertyInputSourceIsEnabled);
    return CFBooleanGetValue((CFBooleanRef)capsLockState);
}

JNIEXPORT void JNICALL Java_com_vbuser_ime_IMEController_toggleIME
(JNIEnv* env, jclass cls, jboolean enable) {
    bool currentState = getCapsLockState();
    
    if (currentState != enable) {
        CGEventSourceRef source = CGEventSourceCreate(kCGEventSourceStateHIDSystemState);
        if (source) {
            CGEventRef keyDown = CGEventCreateKeyboardEvent(source, kVK_CapsLock, true);
            CGEventRef keyUp = CGEventCreateKeyboardEvent(source, kVK_CapsLock, false);
            
            CGEventPost(kCGHIDEventTap, keyDown);
            CGEventPost(kCGHIDEventTap, keyUp);
            
            CFRelease(keyDown);
            CFRelease(keyUp);
            CFRelease(source);
        }
    }
}