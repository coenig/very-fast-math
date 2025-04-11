//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2024 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "failable.h"

#ifdef _WIN32
   #include <comdef.h>
   #include <Wbemidl.h>
   #include <locale>
   #include <string>
   #include <iostream>
   #include <sstream>
   #include <signal.h>
   #pragma comment(lib, "wbemuuid.lib")
   #include <Windows.h>
#elif __linux__
#else
#endif

#include <set>

namespace vfm {

class Process : public Failable {
public:
   Process() : Failable("Process-Helper") {}

   inline bool killByPID(const int pid)
   {
      bool res{ false };

#ifdef _WIN32
      HANDLE hProcess = OpenProcess(PROCESS_TERMINATE, FALSE, pid);

      if (hProcess != NULL) {
         res = TerminateProcess(hProcess, 0);

         if (res) {
            addNote("Process with ID " + std::to_string(pid) + " killed successfully.");
         }
         else {
            addError("Failed to kill process with ID " + std::to_string(pid) + ".");
         }

         CloseHandle(hProcess);
      }
      else {
         addError("Failed to open process with ID " + std::to_string(pid) + ".");
      }

#elif __linux__
      addError("TODO: No unix implementation for killByPID() available.");
#else
      addError("TODO: No implementation for killByPID() available.");
#endif

      return res;
   }

   inline std::set<int> getPIDs(const std::string& command_line_str)
   {
      std::set<int> pids{};

#ifdef _WIN32
      std::wstringstream wss;
      wss << command_line_str.c_str();
      std::wstring wstr = wss.str();

      // Initialize COM
      HRESULT hres_ = CoInitializeEx(0, COINIT_MULTITHREADED);

      //if (FAILED(hres_)) {
      //   std::cerr << "Failed to initialize COM library. Error code: " << hres_ << std::endl;
      //   return -1;
      //}
   
      // Initialize security
      hres_ = CoInitializeSecurity(
         NULL,
         -1,
         NULL,
         NULL,
         RPC_C_AUTHN_LEVEL_DEFAULT,
         RPC_C_IMP_LEVEL_IMPERSONATE,
         NULL,
         EOAC_NONE,
         NULL
      );
   
      //if (FAILED(hres_)) {
      //   std::cerr << "Failed to initialize security. Error code: " << hres_ << std::endl;
      //   CoUninitialize();
      //   return -1;
      //}
   
      // Obtain the initial locator to WMI
      IWbemLocator* pLoc = NULL;
      hres_ = CoCreateInstance(
         CLSID_WbemLocator,
         0,
         CLSCTX_INPROC_SERVER,
         IID_IWbemLocator,
         (LPVOID*)&pLoc
      );
   
      //if (FAILED(hres_)) {
      //   std::cerr << "Failed to create IWbemLocator object. Error code: " << hres_ << std::endl;
      //   CoUninitialize();
      //   return -1;
      //}

      // Connect to WMI through the IWbemLocator::ConnectServer method
      IWbemServices* pSvc = NULL;
      hres_ = pLoc->ConnectServer(
         _bstr_t(L"ROOT\\CIMV2"),
         NULL,
         NULL,
         0,
         NULL,
         0,
         0,
         &pSvc
      );

      //if (FAILED(hres_)) {
      //   std::cerr << "Failed to connect to WMI. Error code: " << hres_ << std::endl;
      //   pLoc->Release();
      //   CoUninitialize();
      //   return -1;
      //}

      // Set the security levels on the proxy
      hres_ = CoSetProxyBlanket(
         pSvc,
         RPC_C_AUTHN_WINNT,
         RPC_C_AUTHZ_NONE,
         NULL,
         RPC_C_AUTHN_LEVEL_CALL,
         RPC_C_IMP_LEVEL_IMPERSONATE,
         NULL,
         EOAC_NONE
      );

      //if (FAILED(hres_)) {
      //   std::cerr << "Failed to set proxy blanket. Error code: " << hres_ << std::endl;
      //   pSvc->Release();
      //   pLoc->Release();
      //   CoUninitialize();
      //   return -1;
      //}

      // Query for processes
      IEnumWbemClassObject* pEnumerator = NULL;
      hres_ = pSvc->ExecQuery(
         bstr_t("WQL"),
         bstr_t("SELECT * FROM Win32_Process"),
         WBEM_FLAG_FORWARD_ONLY | WBEM_FLAG_RETURN_IMMEDIATELY,
         NULL,
         &pEnumerator
      );

      //if (FAILED(hres_)) {
      //   std::cerr << "Failed to execute WQL query. Error code: " << hres_ << std::endl;
      //   pSvc->Release();
      //   pLoc->Release();
      //   CoUninitialize();
      //   return -1;
      //}

      // Enumerate the processes
      IWbemClassObject* pclsObj = NULL;
      ULONG uReturn = 0;
      while (pEnumerator) {
         hres_ = pEnumerator->Next(WBEM_INFINITE, 1, &pclsObj, &uReturn);
         if (uReturn == 0) {
            break;
         }
         VARIANT vtProp;
         hres_ = pclsObj->Get(L"CommandLine", 0, &vtProp, 0, 0);
         if (SUCCEEDED(hres_) && vtProp.vt == VT_BSTR && vtProp.bstrVal != NULL) {
            std::wstring commandLine = vtProp.bstrVal;
            if (commandLine.find(wstr) != std::wstring::npos) {
               hres_ = pclsObj->Get(L"ProcessId", 0, &vtProp, 0, 0);
               addNote("Found process " + std::to_string(vtProp.intVal) + " matching '" + command_line_str + "'.");
               addError(commandLine, ErrorLevelEnum::note);
               pids.insert(vtProp.intVal);
            }
            VariantClear(&vtProp);
         }
         pclsObj->Release();
      }

      pSvc->Release();
      pLoc->Release();
      pEnumerator->Release();
      CoUninitialize();
#elif __linux__
      addError("TODO: No unix implementation for getPIDs() available.");
#else
      addError("TODO: No implementation for getPIDs() available.");
#endif

      return pids;
   }
};

} // vfm::process
