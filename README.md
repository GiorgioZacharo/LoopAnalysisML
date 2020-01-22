# LoopAnalysisML

Overview

    The LoopAnalysisML© framework is a plugin of LLVM version (3.8) and consists of an LLVM Analysis Pass that 
    analyzes loops and extracts information regarding their loop unrolling potential in hardware realizations 
    (hardware accelerators). The goal is to use this information as X features to train a Machine Learning 
    classifier in order to perform prediction on the optimal loop unrolling factor of each loop.

    The process is as follows:

    a) Profiling

        The existing tools of LLVM are used to generate an instrumented version of the binary of
        a provided benchmark.

        http://clang.llvm.org/docs/UsersManual.html#profiling-with-instrumentation

        e.g.

         clang -O3 -fprofile-instr-generate bench.c -o bench_instrumented

        The bench.profdata file generated and used to generate the respective *.ir files of the 
        benchmark.

         bench_instrumented $(BENCH_COMMAND_LINE_PARAMETERS)
         llvm-profdata merge -output=$(BENCH).profdata default.profraw

         clang -S -emit-llvm -O3 -fprofile-instr-use=$(BENCH).profdata -o bench.ir bench.c

     
    b) BB Frequency Annotation and Region Identification

        The Analysis Passes are being in use. We are feeding them the *.ir files that were produced
        in the previous step.

         opt -load ~giorgio/llvm_new/build/lib/BBFreqAnnotation.so -O3 -BBFreqAnnotation -stats -S *.ir > *.bbfreq.ll

        Now every Basic Block has the respective frequency annotated in the output *.bbfreq.ll files.
        This information is going to be used next.

         opt -load ~giorgio/llvm_new/build/lib/IdentifyLoopUnrolling.so -IdentifyLoopUnrolling -stats *.bbfreq.ll > /dev/null 

        
        The output from loading this pass provides us with a full analysis of the Regions. For more details see
        Region Identification Pass bellow.


 Makefiles

    There is a Makefile_region file for every benchmark that needs minor modifications
    in order to be used for each of them.

    The Makefile_region is included in the Makefile_orig of every benchmark.
    Makefile_orig is simply a copy of the original Makefile, wich has been slightly modified
    to use the Makefile_region. (to include the Makefile_Region) 

       
    region

        The region rule requires profile. It receives as input the *.ir files and loads the BB 
        frequency Annotation Pass and outputs the *.bbfreq.ll files. In sequence, these file are
        used to load the final Analysis of Region Identification Pass.


        e.g.
        
        make -f Makefile_orig region


Usage

    The Makefile is used as follows:

    e.g.
    
    make -f Makefile_orig profile
    make -f Makefile_orig region


    make -f Makefile_orig sort_regions



   ** Modifications are needed to comply for every benchmark. **


# Authors

Georgios Zacharopoulos <georgios.zacharopoulos@usi.ch>
Date: January, 2020