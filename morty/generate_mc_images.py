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
dummy_result = create_string_buffer(200000)
script = r"""
@{./src/templates/}@.stringToHeap[MY_PATH]
@{@{./examples/exp4_time_2000_dist_4000_mintimeBetweenLC_1/detailed_archive/}@.findFilesRecursively[debug_trace_array_FALSE.txt]}@*.setScriptVar[temp_var, force].nil
@{@{[i]}@.replaceAll[\, /].removeLastFileExtension[/].generateTestCasesPlain[cex-birdseye, debug_trace_array_FALSE, ./examples/exp4_time_2000_dist_4000_mintimeBetweenLC_1/detailed_archive/scaling_info.txt]
}@*.for[[i], @{temp_var}@.scriptVar]
"""
# Test cases´: all or [cex-birdseye/cex-cockpit-only/cex-full/cex-smooth-birdseye/cex-smooth-full/cex-smooth-with-arrows-birdseye/cex-smooth-with-arrows-full/preview/preview2]

dummy_result = morty_lib.expandScript(script.encode('utf-8'), dummy_result, sizeof(dummy_result)).decode()

print(dummy_result)
