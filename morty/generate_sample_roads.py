from ctypes import *
import platform

# Prepare connection to morty lib.
if platform.system() == 'Windows':
    morty_lib = CDLL('./bin/VFM_MAIN_LIB.dll')
else:
    morty_lib = CDLL('./lib/libvfm.so')
morty_lib.expandScript.argtypes = [c_char_p, c_char_p, c_size_t]
morty_lib.expandScript.restype = c_char_p

#    if (!vfm_data_->isVarDeclared("WIDTH_FACTOR_NON_INFINITE")) vfm_data_->addOrSetSingleVal("WIDTH_FACTOR_NON_INFINITE", 1);
#    if (!vfm_data_->isVarDeclared("HEIGHT_FACTOR_NON_INFINITE")) vfm_data_->addOrSetSingleVal("HEIGHT_FACTOR_NON_INFINITE", 7);
#    if (!vfm_data_->isVarDeclared("OFFSET_X_NON_INFINITE")) vfm_data_->addOrSetSingleVal("OFFSET_X_NON_INFINITE", -1000);
#    if (!vfm_data_->isVarDeclared("OFFSET_Y_NON_INFINITE")) vfm_data_->addOrSetSingleVal("OFFSET_Y_NON_INFINITE", 20);
#    if (!vfm_data_->isVarDeclared("DIMENSION_X")) vfm_data_->addOrSetSingleVal("DIMENSION_X", 500);
#    if (!vfm_data_->isVarDeclared("DIMENSION_Y")) vfm_data_->addOrSetSingleVal("DIMENSION_Y", 600);

# Create EnvModels.
dummy_result = create_string_buffer(200000)
script = r"""            
@{@WIDTH_FACTOR_NON_INFINITE = 1}@.eval
@{@HEIGHT_FACTOR_NON_INFINITE = 1}@.eval
@{@OFFSET_X_NON_INFINITE = 0}@.eval
@{@OFFSET_Y_NON_INFINITE = 3}@.eval
@{@DIMENSION_X = 50}@.eval
@{@DIMENSION_Y = 6}@.eval
@{((0, 0), 0, ( 1, 2, 4, 2000 ) ))}@.createRoadGraph[0]
@{0}@.storeRoadGraph[test1.png]
@{((0, 0), 0, ( 2, 2, 4, 1000 ) ))}@.createRoadGraph[0]
@{0}@.storeRoadGraph[test2.png]
@{((0, 0), 0, ( 3, 2, 4, 500 ) ))}@.createRoadGraph[0]
@{0}@.storeRoadGraph[test3.png]
@{((0, 0), 0, ( 4, 2, 4, 200 ) ))}@.createRoadGraph[0]
@{0}@.storeRoadGraph[test4.png]
@{((0, 0), 0, ( 5, 2, 4, 100 ) ))}@.createRoadGraph[0]
@{0}@.storeRoadGraph[test5.png]
@{((0, 0), 0, ( 6, 2, 4, 50 ) ))}@.createRoadGraph[0]
@{0}@.storeRoadGraph[test6.png]
@{((0, 0), 0, ( 7, 2, 4, 20 ) ))}@.createRoadGraph[0]
@{0}@.storeRoadGraph[test7.png]
      """
# Test cases´: all or [cex-birdseye/cex-cockpit-only/cex-full/cex-smooth-birdseye/cex-smooth-full/cex-smooth-with-arrows-birdseye/cex-smooth-with-arrows-full/preview/preview2]

dummy_result = morty_lib.expandScript(script.encode('utf-8'), dummy_result, sizeof(dummy_result)).decode()

print(dummy_result)
