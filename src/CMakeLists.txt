# ============================================================================================================
# C O P Y R I G H T
# ------------------------------------------------------------------------------------------------------------
# copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
# ============================================================================================================

cmake_minimum_required(VERSION 3.11.0)

set(VFM_MAIN_SOURCES
   data_pack.cpp
   dat_src_arr_as_float_vector.cpp
   dat_src_arr.cpp
   dat_src_arr_as_random_access_file.cpp
   dat_src_arr_as_readonly_random_access_file.cpp
   dat_src_arr_as_readonly_string.cpp
   dat_src_arr_as_string.cpp
   dat_src_arr_null.cpp
   dat_src_arr_readonly.cpp
   equation.cpp
   fsm_singleton_generator.cpp
   fsm_transition.cpp
   fsm_resolver.cpp
   fsm_resolver_default.cpp
   fsm_resolver_remain_on_no_transition.cpp
   fsm_resolver_remain_on_no_transition_and_obey_insertion_order.cpp
   fsm_resolver_default_max_trans_weight.cpp
   fsm_resolver_factory.cpp
   math_struct.cpp
   meta_rule.cpp
   mc_types.cpp
   operator_structure.cpp
   parser.cpp
   failable.cpp
   static_helper.cpp
   term.cpp
   term_abs.cpp
   term_arith.cpp
   term_array.cpp
   term_compound.cpp
   term_compound_operator.cpp
   term_const.cpp
   term_div.cpp
   term_array_length.cpp
   term_logic.cpp
   term_logic_and.cpp
   term_logic_eq.cpp
   term_logic_gr.cpp
   term_logic_greq.cpp
   term_logic_neq.cpp
   term_logic_or.cpp
   term_logic_sm.cpp
   term_logic_smeq.cpp
   term_delete.cpp
   term_malloc.cpp
   term_meta_compound.cpp
   term_meta_simplification.cpp
   term_optional.cpp
   term_anyway.cpp
   term_minus.cpp
   term_mod.cpp
   term_max.cpp
   term_min.cpp
   term_mult.cpp
   term_neg.cpp
   term_plus.cpp
   term_pow.cpp
   term_ln.cpp
   terms_trigonometry.cpp
   term_rand.cpp
   term_trunc.cpp
   term_val.cpp
   term_var.cpp
   term_set_var.cpp
   term_set_arr.cpp
   term_get_arr.cpp
   term_while_limited.cpp
   term_sqrt.cpp
   term_rsqrt.cpp
   term_func_ref.cpp
   term_func_eval.cpp
   term_func_lambda.cpp
   term_literal.cpp
   test_functions.cpp
   images.cpp
   model_checker.cpp
   type_abstraction_layer.cpp
   term_fctin.cpp
   term_print.cpp
   term_id.cpp
   cryptor_encoder.cpp
   cryptor_rule.cpp
   cryptor_starter.cpp
   cryptor_tester.cpp
   cpp_type_struct.cpp
   cpp_type_atomic.cpp
   cpp_type_enum.cpp
   cpp_parser.cpp
   earley_parser.cpp
   earley_edge.cpp
   earley_grammar.cpp
   earley_parse_tree.cpp
   earley_recognizer.cpp
   earley_rec_rule.cpp
   yaaa_runnable.cpp
   yaaa_activity.cpp
   yaaa_graph.cpp
   ../include/model_checking/cex_processing/mc_trajectory_generator.cpp
   ../include/model_checking/cex_processing/mc_trajectory_configs.cpp
   ../include/model_checking/cex_processing/mc_trajectory_config_helpers.cpp
   ../include/model_checking/cex_processing/mc_trajectory_to_gif.cpp
   ../include/model_checking/cex_processing/mc_trajectory_to_osc.cpp
   ../include/model_checking/cex_processing/mc_visualization_launchers.cpp
   smv_module.cpp
   smv_module_elements.cpp
   script.cpp
   environment_model_generator.cpp
   highway_image.cpp
   code_block.cpp
   interactive_testing.cpp
   options.cpp
   gui.cpp
   interpreter_terminal.cpp
   custom_widgets.cpp
   road_graph.cpp
   xml_generator.cpp
haru/hpdf_3dmeasure.c
haru/hpdf_annotation.c
haru/hpdf_array.c
haru/hpdf_binary.c
haru/hpdf_boolean.c
haru/hpdf_catalog.c
haru/hpdf_destination.c
haru/hpdf_dict.c
haru/hpdf_direct.c
haru/hpdf_doc.c
haru/hpdf_doc_png.c
haru/hpdf_encoder.c
haru/hpdf_encoder_cns.c
haru/hpdf_encoder_cnt.c
haru/hpdf_encoder_jp.c
haru/hpdf_encoder_kr.c
haru/hpdf_encoder_utf.c
haru/hpdf_encrypt.c
haru/hpdf_encryptdict.c
haru/hpdf_error.c
haru/hpdf_exdata.c
haru/hpdf_ext_gstate.c
haru/hpdf_font.c
haru/hpdf_font_cid.c
haru/hpdf_font_tt.c
haru/hpdf_font_type1.c
haru/hpdf_fontdef.c
haru/hpdf_fontdef_base14.c
haru/hpdf_fontdef_cid.c
haru/hpdf_fontdef_cns.c
haru/hpdf_fontdef_cnt.c
haru/hpdf_fontdef_jp.c
haru/hpdf_fontdef_kr.c
haru/hpdf_fontdef_tt.c
haru/hpdf_fontdef_type1.c
haru/hpdf_gstate.c
haru/hpdf_image.c
haru/hpdf_image_ccitt.c
haru/hpdf_image_png.c
haru/hpdf_info.c
haru/hpdf_list.c
haru/hpdf_mmgr.c
haru/hpdf_name.c
haru/hpdf_namedict.c
haru/hpdf_null.c
haru/hpdf_number.c
haru/hpdf_objects.c
haru/hpdf_outline.c
haru/hpdf_page_label.c
haru/hpdf_page_operator.c
haru/hpdf_pages.c
haru/hpdf_pdfa.c
haru/hpdf_real.c
haru/hpdf_shading.c
haru/hpdf_streams.c
haru/hpdf_string.c
haru/hpdf_u3d.c
haru/hpdf_utils.c
haru/hpdf_xref.c
    # examples/hello_world_environment.cpp
)

if(UNIX)
    set(FLTK_BUILD_SHARED_LIBS ON)
    
    add_library(
        VFM_MAIN_LIB SHARED
        ${VFM_MAIN_SOURCES}
    )

    target_link_libraries(VFM_MAIN_LIB fltk fltk_images stdc++fs)
else()
    add_library(
        VFM_MAIN_LIB STATIC
        ${VFM_MAIN_SOURCES}
     "interpreter_terminal.cpp")
endif()

add_subdirectory(fltk)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/fltk fltk)

add_executable(
   vfm
   ${ASMJIT_SRC} main.cpp
 "interpreter_terminal.cpp")

target_link_libraries(
   vfm
   fltk
   fltk_images
   ${ASMJIT_DEPS}
   VFM_MAIN_LIB
)

if(MSVC)
   target_link_options(VFM_MAIN_LIB PRIVATE "/STACK:100000000") # Use to increase stack size on win
   target_link_options(vfm PRIVATE "/STACK:100000000")          # Use to increase stack size on win
endif()

if(UNIX)
    set_target_properties(
        vfm
        PROPERTIES OUTPUT_NAME "vfm"
    )
    set_target_properties(
        VFM_MAIN_LIB
        PROPERTIES OUTPUT_NAME "vfm"
    )
endif()

install(
    TARGETS VFM_MAIN_LIB
    DESTINATION lib
)
install(
    DIRECTORY include/
    DESTINATION include/vfm
)
