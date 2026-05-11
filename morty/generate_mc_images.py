from ctypes import *
import platform

# Prepare connection to morty lib.
if platform.system() == 'Windows':
    morty_lib = CDLL('./bin/VFM_MAIN_LIB.dll')
else:
    morty_lib = CDLL('./lib/libvfm.so')
morty_lib.expandScript.argtypes = [c_char_p, c_char_p, c_size_t]
morty_lib.expandScript.restype = c_char_p

# Create EnvModels.
dummy_result = create_string_buffer(20000)
script_envmodels = r"""
@{./src/templates/}@.stringToHeap[MY_PATH]
@{@{examples/gp/detailed_archive/run_0}@.findFilesRecursively[debug_trace_array.txt]}@*.setScriptVar[temp_var, force].nil
@{@{[i]}@.replaceAll[\, /].removeLastFileExtension[/].generateTestCasesPlain[cex-birdseye]
}@*.for[[i], @{temp_var}@.scriptVar]
"""
dummy_result = morty_lib.expandScript(script_envmodels.encode('utf-8'), dummy_result, sizeof(dummy_result)).decode()

print(dummy_result)
