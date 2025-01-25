#include "com_vbuser_ime_IMEController.h"
#include <windows.h>
#include <tchar.h>
#include <iostream>
#include <objbase.h>
#include <msctf.h> // TSF Header

#pragma comment(lib, "ole32.lib")

void SetInputLanguage(BOOL enableEnglish) {
    HRESULT hr = CoInitialize(NULL); // 初始化 COM 库
    if (FAILED(hr)) {
        std::cerr << "Failed to initialize COM library." << std::endl;
        return;
    }

    ITfInputProcessorProfiles* pProfiles = NULL;
    // 创建 ITfInputProcessorProfiles 对象
    hr = CoCreateInstance(CLSID_TF_InputProcessorProfiles, NULL, CLSCTX_INPROC_SERVER,
                          IID_ITfInputProcessorProfiles, (void**)&pProfiles);
    if (SUCCEEDED(hr) && pProfiles != NULL) {
        // 英文语言标识符 (LANGID)
        LANGID langId = enableEnglish ? MAKELANGID(LANG_ENGLISH, SUBLANG_ENGLISH_US)
                                      : MAKELANGID(LANG_CHINESE_SIMPLIFIED, SUBLANG_CHINESE_SIMPLIFIED);

        // 激活对应的输入法语言
        hr = pProfiles->ChangeCurrentLanguage(langId);
        if (SUCCEEDED(hr)) {
            std::cout << (enableEnglish ? "Switched to English input method."
                                        : "Switched to Chinese input method.") << std::endl;
        } else {
            std::cerr << "Failed to switch input method." << std::endl;
        }

        pProfiles->Release(); // 释放 ITfInputProcessorProfiles 对象
    } else {
        std::cerr << "Failed to create ITfInputProcessorProfiles instance." << std::endl;
    }

    CoUninitialize(); // 释放 COM 库
}

// JNI 接口实现
JNIEXPORT void JNICALL Java_com_vbuser_ime_IMEController_toggleIME
(JNIEnv* env, jclass cls, jboolean enable) {
    SetInputLanguage(enable); // 切换输入法
}