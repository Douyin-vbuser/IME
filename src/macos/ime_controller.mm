#include "com_vbuser_ime_IMEController.h"
#import <Foundation/Foundation.h>
#import <Carbon/Carbon.h>

void SelectEnglishKeyboardLayout() {
    CFArrayRef sourceList = TISCreateInputSourceList(NULL, false);
    if (sourceList) {
        for (CFIndex i = 0; i < CFArrayGetCount(sourceList); ++i) {
            TISInputSourceRef source = (TISInputSourceRef)CFArrayGetValueAtIndex(sourceList, i);
            CFStringRef sourceType = (CFStringRef)TISGetInputSourceProperty(source, kTISPropertyInputSourceType);

            if (CFEqual(sourceType, kTISTypeKeyboardLayout)) {
                CFArrayRef languages = (CFArrayRef)TISGetInputSourceProperty(source, kTISPropertyInputSourceLanguages);
                if (languages && CFArrayGetCount(languages) > 0) {
                    CFStringRef langCode = (CFStringRef)CFArrayGetValueAtIndex(languages, 0);
                    if (CFStringCompare(langCode, CFSTR("en"), 0) == kCFCompareEqualTo) {
                        TISSelectInputSource(source);
                        break;
                    }
                }
            }
        }
        CFRelease(sourceList);
    }
}

JNIEXPORT void JNICALL Java_com_vbuser_ime_IMEController_toggleIME
  (JNIEnv* env, jclass cls, jboolean enable) {
    if (enable) {
        SelectEnglishKeyboardLayout();
    } else {
    }
}