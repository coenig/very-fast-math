# ============================================================================================================
# C O P Y R I G H T
# ------------------------------------------------------------------------------------------------------------
# copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
# ============================================================================================================

#dc_include(./asmjit/CMakeLists.txt)

dc_add_to_target(
INCLUDE_IF

SOURCES
   # asmjit/src/asmjit/asmjit.h
   # asmjit/src/asmjit/asmjit_apibegin.h
   # asmjit/src/asmjit/asmjit_apiend.h
   # asmjit/src/asmjit/asmjit_build.h
   # asmjit/src/asmjit/base.h
   # asmjit/src/asmjit/arm.h
   # asmjit/src/asmjit/x86.h
   # asmjit/src/asmjit/base/arch.cpp
   # asmjit/src/asmjit/base/arch.h
   # asmjit/src/asmjit/base/assembler.cpp
   # asmjit/src/asmjit/base/assembler.h
   # asmjit/src/asmjit/base/codebuilder.cpp
   # asmjit/src/asmjit/base/codebuilder.h
   # asmjit/src/asmjit/base/codecompiler.cpp
   # asmjit/src/asmjit/base/codecompiler.h
   # asmjit/src/asmjit/base/codeemitter.cpp
   # asmjit/src/asmjit/base/codeemitter.h
   # asmjit/src/asmjit/base/codeholder.cpp
   # asmjit/src/asmjit/base/codeholder.h
   # asmjit/src/asmjit/base/constpool.cpp
   # asmjit/src/asmjit/base/constpool.h
   # asmjit/src/asmjit/base/cpuinfo.cpp
   # asmjit/src/asmjit/base/cpuinfo.h
   # asmjit/src/asmjit/base/func.cpp
   # asmjit/src/asmjit/base/func.h
   # asmjit/src/asmjit/base/globals.cpp
   # asmjit/src/asmjit/base/globals.h
   # asmjit/src/asmjit/base/inst.cpp
   # asmjit/src/asmjit/base/inst.h
   # asmjit/src/asmjit/base/logging.cpp
   # asmjit/src/asmjit/base/logging.h
   # asmjit/src/asmjit/base/misc_p.h
   # asmjit/src/asmjit/base/operand.cpp
   # asmjit/src/asmjit/base/operand.h
   # asmjit/src/asmjit/base/osutils.cpp
   # asmjit/src/asmjit/base/osutils.h
   # asmjit/src/asmjit/base/regalloc.cpp
   # asmjit/src/asmjit/base/regalloc_p.h
   # asmjit/src/asmjit/base/runtime.cpp
   # asmjit/src/asmjit/base/runtime.h
   # asmjit/src/asmjit/base/simdtypes.h
   # asmjit/src/asmjit/base/string.cpp
   # asmjit/src/asmjit/base/string.h
   # asmjit/src/asmjit/base/utils.cpp
   # asmjit/src/asmjit/base/utils.h
   # asmjit/src/asmjit/base/vmem.cpp
   # asmjit/src/asmjit/base/vmem.h
   # asmjit/src/asmjit/base/zone.cpp
   # asmjit/src/asmjit/base/zone.h
   # asmjit/src/asmjit/x86/x86assembler.cpp
   # asmjit/src/asmjit/x86/x86assembler.h
   # asmjit/src/asmjit/x86/x86builder.cpp
   # asmjit/src/asmjit/x86/x86builder.h
   # asmjit/src/asmjit/x86/x86compiler.cpp
   # asmjit/src/asmjit/x86/x86compiler.h
   # asmjit/src/asmjit/x86/x86emitter.h
   # asmjit/src/asmjit/x86/x86globals.h
   # asmjit/src/asmjit/x86/x86internal.cpp
   # asmjit/src/asmjit/x86/x86internal_p.h
   # asmjit/src/asmjit/x86/x86inst.cpp
   # asmjit/src/asmjit/x86/x86inst.h
   # asmjit/src/asmjit/x86/x86instimpl.cpp
   # asmjit/src/asmjit/x86/x86instimpl_p.h
   # asmjit/src/asmjit/x86/x86logging.cpp
   # asmjit/src/asmjit/x86/x86logging_p.h
   # asmjit/src/asmjit/x86/x86misc.h
   # asmjit/src/asmjit/x86/x86operand.cpp
   # asmjit/src/asmjit/x86/x86operand_regs.cpp
   # asmjit/src/asmjit/x86/x86operand.h
   # asmjit/src/asmjit/x86/x86regalloc.cpp
   # asmjit/src/asmjit/x86/x86regalloc_p.h
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
   # main.cpp
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
   term_malloc.cpp
   term_delete.cpp
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
   term_get_arr.cpp
   term_set_arr.cpp
   term_while_limited.cpp
   term_sqrt.cpp
   term_rsqrt.cpp
   term_func_ref.cpp
   term_func_eval.cpp
   term_func_lambda.cpp
   term_literal.cpp
   term_fctin.cpp
   term_print.cpp
   term_id.cpp
   images.cpp
   model_checker.cpp
   type_abstraction_layer.cpp
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

TESTS
)
