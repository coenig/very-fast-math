//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "cryptor/cryptor_tester.h"
#include "math_struct.h"
#include "equation.h"
#include "data_pack.h"

//#if defined(ASMJIT_ENABLED)
//   #include "asmjit.h"
//   using namespace asmjit;
//#endif

using namespace vfm;

// Signature of the generated function.
//typedef float(*Func)(float, float);

//#if defined(ASMJIT_ENABLED)
//float CryptorTester::testJitStatic() {
//	JitRuntime rt;                          // Runtime specialized for JIT code execution.
//
//	CodeHolder code;                        // Holds code and relocation information.
//	code.init(rt.getCodeInfo());            // Initialize to the same arch as JIT runtime.
//
//	x86::Compiler cc(&code);                  // Create and attach x86::Compiler to `code`.
//	cc.addFunc(FuncSignature2<float, float, float>());
//
//	X86Xmm x = cc.newXmm("x");
//	X86Xmm y = cc.newXmm("y");
//	cc.setArg(0, x);
//	cc.setArg(1, y);
//	cc.divsd(x, y);
//	cc.ret(x);
//
//	cc.endFunc();                           // End of the function body.
//	cc.finalize();                          // Translate and assemble the whole `cc` content.
//											// ----> x86::Compiler is no longer needed from here and can be destroyed <----
//
//	Func fn;
//	Error err = rt.add(&fn, &code);         // Add the generated code to the runtime.
//	if (err) return 1;                      // Handle a possible error returned by AsmJit.
//											// ----> CodeHolder is no longer needed from here and can be destroyed <----
//
//	float result = fn(7000.7, 30.6);       // Execute the generated code.
//	printf("%f\n", result);
//
//	rt.release(fn);                         // RAII, but let's make it explicit.
//
//	return 0.0;
//}
//#endif

// std::shared_ptr<vfm::DataPack> mp = std::make_shared<vfm::DataPack>();

// void fill_map() {
// 	std::string cod = "Dies hier ist kodiert.";
// 	std::string dec = "Dies hier ist dekodiert.";
// 	float x = 4.5, y = 1, z = 3, a = 0, b = 7, c = 3.6, d = 4.7, e = 300;
// 	mp->addOrSetSingleVal("x", x);
// 	mp->addOrSetSingleVal("y", y);
// 	mp->addOrSetSingleVal("z", z);
// 	mp->addOrSetSingleVal("a", a);
// 	mp->addOrSetSingleVal("b", b);
// 	mp->addOrSetSingleVal("c", c);
// 	mp->addOrSetSingleVal("d", d);
// 	mp->addOrSetSingleVal("e", e);
// 	mp->addStringAsArray("C", cod);
// 	mp->addReadonlyStringAsArray("D", dec);
// }

void CryptorTester::editor(int argc, char* argv[])
{
   std::string formula{};
   std::string resolve{};
   std::shared_ptr<MathStruct> m{};

	for (; formula != "exit";) {
		std::cout << "Enter a formula: ";
		std::getline(std::cin, formula);
		m = MathStruct::parseMathStruct(formula, true);
		m->setChildrensFathers();
		std::cout << "This is what I parsed: " << *m << std::endl;
		
		if (m->isEquation()) {
			std::cout << "Resolve to: ";
			std::getline(std::cin, resolve);
			try { 
            m->resolveTo([resolve](std::shared_ptr<MathStruct> m) {
               return m->serialize() == resolve; 
            }, true); 
         } catch (const std::exception& ex) { 
            (void) ex; 
         }

			try { 
            m->resolveTo([resolve](std::shared_ptr<MathStruct> m) {
               return m->serialize() == resolve; 
            }, false);
         } catch (const std::exception & ex) { 
            (void) ex; 
         }

			std::cout << "This is the result: " << *m << std::endl;
		}
	}
}

// void CryptorTester::encryptor(int argc, const char* argv[])
// {
// 	CryptorStarter CryptorStarter(argc, argv);

// 	//std::shared_ptr<dat_src_arr> enc_f = std::make_shared<dat_src_arr_as_random_access_file>("DistProtoManager.stable.zip.cry", true);
// 	//std::shared_ptr<dat_src_arr> dec_f = std::make_shared<dat_src_arr_as_readonly_random_access_file>("DistProtoManager.stable.zip");
// 	std::shared_ptr<DataSrcArray> enc_f2 = std::make_shared<DataSrcArrayAsReadonlyRandomAccessFile>("./examples/Project2.rar.4.cry");
// 	std::shared_ptr<DataSrcArray> dec_f2 = std::make_shared<DataSrcArrayAsRandomAccessFile>("./examples/Project2.rar.4.cry.rar", true);

// 	//const std::shared_ptr<dat_src_arr> enc_s = std::make_shared<dat_src_arr_as_string>(100);
// 	//const std::shared_ptr<dat_src_arr> enc_s_2 = std::make_shared<dat_src_arr_as_readonly_string>("Dies ist Mein Test.");
// 	//const std::shared_ptr<dat_src_arr> dec_s = std::make_shared<dat_src_arr_as_readonly_string>("Dies ist ein Test.");
// 	//const std::shared_ptr<dat_src_arr> dec_s_2 = std::make_shared<dat_src_arr_as_string>(100);

// 	//CryptorStarter.get_encryptor()->load_from_file("./examples/lukes_crypt.enc");
// 	//CryptorStarter.get_decryptor()->load_from_file("./examples/lukes_crypt.enc.autogenerated.dec");
// 	//CryptorStarter.auto_create_decoder();

// 	std::cout << CryptorStarter << std::endl;

// 	//std::cout << "--- D ==> C ---";
// 	//CryptorStarter.encrypt(dec_f, enc_f);
// 	//std::cout << std::endl << "This is the map I used:" << std::endl << CryptorStarter.get_data();

// 	//debug_output = 1;
// 	std::cout << std::endl << "--- C ==> D ---";
// 	CryptorStarter.decrypt(enc_f2, dec_f2);
// 	//std::cout << std::endl << "This is the map I used:" << std::endl << CryptorStarter.get_data();

// 	//CryptorStarter.test_many_random(2, 2, 100, 1000);
// }

// void CryptorTester::testJitBenchmark(const std::shared_ptr<DataPack> d, std::shared_ptr<MathStruct>& m, int& num) 
// {
// 	MathStruct::randomize_rand_nums();
// 	float res_eval = m->eval(d);

// 	std::cout << "---------- Benchmark testing regular eval vs. JIT eval ----------" << std::endl;
// 	std::cout << "This is the data: " << std::endl << d << std::endl;
// 	std::cout << "Formula:         " << *m << std::endl;
// 	std::cout << "Eval result:     " << res_eval << std::endl;
// 	float res_jit = m->evalJit();
// 	std::cout << "JIT Eval result: " << res_jit << "           <" << (res_eval == res_jit ? "OK" : "FAILED") << ">" << std::endl;

// 	clock_t begin1 = clock();
// 	std::cout << "Eval     x " << num << ": ";
// 	for (int i = 0; i < num; i++) m->eval(d);
// 	clock_t end1 = clock();
// 	std::cout << float(end1 - begin1) << " ms." << std::endl;

// 	clock_t begin2 = clock();
// 	std::cout << "JIT Eval x " << num << ": ";
// 	for (int i = 0; i < num; i++) m->evalJit();
// 	clock_t end2 = clock();
// 	std::cout << float(end2 - begin2) << " ms." << std::endl;

// 	std::cout << "-----------------------------------------------------------------" << std::endl;
// }

// void CryptorTester::justATest() {
// 	int num = 1000;
// 	std::shared_ptr<DataPack> d = std::make_shared<DataPack>();

//     d->addOrSetSingleVal("a", 0);
//     d->addArrayAndOrSetArrayVal("H_f", 0, 10.5);
//     d->addArrayAndOrSetArrayVal("H_f", 1, 11.5);
//     d->addArrayAndOrSetArrayVal("H_f", 2, 12.5);
//     d->addArrayAndOrSetArrayVal("H_f", 3, 13.5);
//     d->addArrayAndOrSetArrayVal("H_f", 4, 14.5);

// 	std::shared_ptr<MathStruct> m = MathStruct::parseMathStruct("H_f(a) * 5");
	
// #if defined(ASMJIT_ENABLED)
//    m->createAssembly(d);
// #endif

// 	for (int j = 0; j < 3; j++) {
//         d->addOrSetSingleVal("a", j);
//         CryptorTester::test_jit_benchmark(d, m, num);
//     }
// }

CryptorTester::CryptorTester() {}
