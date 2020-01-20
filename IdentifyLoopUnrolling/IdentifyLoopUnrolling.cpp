//===------------------------- IdentifyLoopUnrolling.cpp -------------------------===//
//
//                     The LLVM Compiler Infrastructure
// 
// This file is distributed under the Università della Svizzera italiana (USI) 
// Open Source License.
//
// Author         : Georgios Zacharopoulos 
// Date Started   : September, 2016
//
//===----------------------------------------------------------------------===//
//
// This file identifies Loop Unrolling Potential in Regions of SW Applications.
//
//===----------------------------------------------------------------------===//

#include "llvm/ADT/Statistic.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/Analysis/RegionPass.h"
#include "llvm/Analysis/RegionInfo.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/DependenceAnalysis.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/RegionIterator.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Analysis/RegionIterator.h"
#include "llvm/Analysis/BlockFrequencyInfo.h"
#include "llvm/Analysis/BlockFrequencyInfoImpl.h"
#include "llvm/Pass.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Debug.h"
#include "llvm/Analysis/TargetLibraryInfo.h"
#include "llvm/Transforms/Utils/Local.h"
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "llvm/IR/CFG.h"
#include "../Identify.h"              // My RegionSeeker header file.

#define DEBUG_TYPE   "IdentifyLoopUnrolling"

#ifndef LOOP_UNROLLING_FACTOR
#define LOOP_UNROLLING_FACTOR     4   // Loop Unrolling factor.
#endif

using namespace llvm;

STATISTIC(LoopCounter, "The # of Loops Identified");
STATISTIC(BBCounter, "The # of Basic Blocks within loops (Loop Bodies) Identified");

namespace {

  struct IdentifyLoopUnrolling : public FunctionPass {
    static char ID; // Pass Identification, replacement for typeid.

    IdentifyLoopUnrolling() : FunctionPass(ID) {}

    bool runOnFunction(Function &F) override {

      // Analysis Required to pick up Info for Regions, Loops and Dep. Analysis.
      //RegionInfo *RI = &getAnalysis<RegionInfoPass>().getRegionInfo();
      LoopInfo *LI = &getAnalysis<LoopInfoWrapperPass>().getLoopInfo();
      DependenceAnalysis *DA = &getAnalysis<DependenceAnalysis>();

      std::vector<Loop *> Loops;    // Vector to hold different loops.
      Loops.clear();                // Clear Vectors.


      errs() << "\n\nFunction Name is : " << F.getName() << "\n";

      // Iterate over BBs of the Function
      for(Function::iterator BB = F.begin(), E = F.end(); BB != E; ++BB) {   

        if (Loop *L = LI->getLoopFor(&*BB) ) {

          // If loop L is not in the list.     
          if (find_loop(Loops, L) == -1) {
     
            Loops.push_back(L);

            errs() << "\n\n     Num of Back Edges         : " << L->getNumBackEdges();
            errs() << "\n     Loop Depth                : " << L->getLoopDepth() << "\n";
            ++LoopCounter;

            PHINode *IndVariable = L->getCanonicalInductionVariable();
            BasicBlockAnalysis(BB);    
          }
        }

      }



      return false;
    }


    // Run Inside BB Function. 
    // Fix details later.
    virtual bool BasicBlockAnalysis(Function::iterator &BB) {

      errs() << "     BB Name                   : " << BB->getName() << " \n";
      errs() << "     ***************************************\n";
      DEBUG(errs() << "I am here!\n");

      unsigned int NumberOfLoads = 0, NumberOfStores = 0, NumberOfLoadsReused=0, NumberOfLoopCarriedDeps=0;
      float CriticalPath =0, LoopUnrolledCriticalPath = 0;
      unsigned int CriticalPathCycles =0, LoopUnrolledCriticalPathCycles = 0;

      // Get the Number of Loads and Stores in the BB.
      NumberOfLoads  = getLoads(&*BB);
      NumberOfStores = getStores(&*BB);

      // Check if there are Loop Carried Dependencies in this BB.
      NumberOfLoopCarriedDeps= getLoopCarriedDependencies(&*BB);

      if (NumberOfLoopCarriedDeps> 1)
        NumberOfLoadsReused = getNumberOfLoadsReused(&*BB);

      // Compute the Cr. Path of BB (Body Loop) in Nsecs.
      CriticalPath = getDelayOfBB(&*BB);

      // Compute the Unrolled Cr. Path of the Loop in NSecs.
      if (NumberOfLoopCarriedDeps > 0 && NumberOfLoopCarriedDeps > NumberOfLoadsReused)
        LoopUnrolledCriticalPath = CriticalPath * LOOP_UNROLLING_FACTOR;
      else
        LoopUnrolledCriticalPath = CriticalPath;

      // Cyclification.
      CriticalPathCycles = ceil(CriticalPath/NSECS_PER_CYCLE);
      LoopUnrolledCriticalPathCycles = ceil(LoopUnrolledCriticalPath/NSECS_PER_CYCLE);


      // Print Info
      errs() << "     ***************************************\n";
      errs() << "     Number Of New  Loads      : " << NumberOfLoads << " \n";
      errs() << "     Number Of Stores          : " << NumberOfStores << " \n";
      errs() << "     Number Of Loads Reused    : " << NumberOfLoadsReused    << " \n";
      if (NumberOfLoopCarriedDeps > 0 && NumberOfLoopCarriedDeps > NumberOfLoadsReused )
        errs() << "     Loop Carried Dependencies : " << NumberOfLoopCarriedDeps << "\n";
      if (NumberOfLoadsReused)
        errs() << "     Data Reuse! \n";
      // else
      //   errs() << "     No Data Reuse! \n"; 

      errs() << "\n     Critical Path of Body Loop             : " << format("%.8f", CriticalPath) 
            << "  nSecs |\t" << CriticalPathCycles  <<"\tCycle(s)\n";
      errs() << "     Critical Path  Unrolled Loop by    " << LOOP_UNROLLING_FACTOR << "   : " << 
              format("%.8f", LoopUnrolledCriticalPath) << "  nSecs |\t" << LoopUnrolledCriticalPathCycles  << "\tCycle(s)\n";


      ++BBCounter;
      return false;
    }


    unsigned int getLoopCarriedDependencies(BasicBlock *BB) {

      unsigned int LoopCarriedDep = 0;
      bool IndVariableFound = false;

      DependenceAnalysis *DA = &getAnalysis<DependenceAnalysis>();

      for(BasicBlock::iterator BI = BB->begin(), BE = BB->end(); BI != BE; ++BI) {

        if (PHINode *PN  = dyn_cast<PHINode>(&*BI)) {

          for (unsigned int i =0; i < PN->getNumIncomingValues(); i++) {
            if (PN->getIncomingBlock(i) == BB) {
              LoopCarriedDep++;
              errs() << "     Phi : " << *PN << "\n";
              Value *DependencyVar = PN->getIncomingValueForBlock(BB);

              if (!IndVariableFound)
                if (isInductionVariable(DependencyVar)){
                  LoopCarriedDep--;
                  IndVariableFound = true;
                  errs() << "     Induction Variable Found! " << "\n";
                }

              // Instruction * DepInstr = dyn_cast<Instruction>(DependencyVar);

              // std::unique_ptr<Dependence> Dep = DA->depends( BI, BI, false);

              // if  (Dep->isInput())
              //   errs() << " True Dependence! \n " ;

            }
          }
        }
      }

      return LoopCarriedDep;
    }

    // Detect potential Data reuse. Used for Corner Cases though.
    unsigned int getNumberOfLoadsReused(BasicBlock *BB) {

      unsigned int LoadsReused = 0;

      for(BasicBlock::iterator BI = BB->begin(), BE = BB->end(); BI != BE; ++BI) {

        if (PHINode *PN  = dyn_cast<PHINode>(&*BI))
          isLoadtheIncomingValueofPhi(PN, BB, LoadsReused);                 

      }

      return LoadsReused;
    }

    void isLoadtheIncomingValueofPhi(PHINode *PN, BasicBlock *BB, unsigned int &LoadsReused) {

      //bool LoadsReused = false;

      Value *incoming_value = PN->getIncomingValueForBlock(BB);

      if (LoadInst *Load = dyn_cast<LoadInst>(incoming_value)) {
        errs() << "     Load Reused   : " << *Load << "\n";
        LoadsReused++;
      }

      if (PHINode *PN_weird = dyn_cast<PHINode>(incoming_value))
        isLoadtheIncomingValueofPhi(PN_weird, BB, LoadsReused );

      //return LoadsReused;

    }

    bool isInductionVariable(Value *DependencyVar) {

      if (Operator *DependencyInstr = dyn_cast<Operator>(DependencyVar)) {

        if (DependencyInstr->getOpcode() == Instruction::Add) {

          if (ConstantInt *constant = dyn_cast<ConstantInt>(DependencyInstr->getOperand(1)) )
            if (constant->getSExtValue() == 1 || constant->getSExtValue() == -1 )
              return true;
        }
      }

      return false; 
    }


    unsigned int getLoads(BasicBlock *BB) {

      int NumberOfLoads = 0;

      for(BasicBlock::iterator BI = BB->begin(), BE = BB->end(); BI != BE; ++BI)
        if(LoadInst *Load = dyn_cast<LoadInst>(&*BI)) {
          errs() << "     New Load   : " << *Load << "\n";
          NumberOfLoads++;
        }

      return NumberOfLoads;  
    }

    unsigned int getStores(BasicBlock *BB) {

      int NumberOfStores = 0;

      for(BasicBlock::iterator BI = BB->begin(), BE = BB->end(); BI != BE; ++BI)
        if(StoreInst *Store = dyn_cast<StoreInst>(&*BI))
          NumberOfStores++;

      return NumberOfStores;  
    }

    int find_loop(std::vector<Loop *> loop_list, Loop *Loop) {

      for (unsigned i = 0; i < loop_list.size(); i++)
        if (loop_list[i] == Loop)
          return i;
      
    
      return -1;
    }
    

    virtual void getAnalysisUsage(AnalysisUsage& AU) const override {
              
        AU.addRequired<LoopInfoWrapperPass>();
        AU.addRequired<RegionInfoPass>();
        AU.addRequired<DependenceAnalysis>();
        AU.addRequiredTransitive<RegionInfoPass>();
        AU.addRequiredTransitive<ScalarEvolutionWrapperPass>();
        AU.addRequired<BlockFrequencyInfoWrapperPass>();
        AU.setPreservesAll();
    } 
  };
}

char IdentifyLoopUnrolling::ID = 0;
static RegisterPass<IdentifyLoopUnrolling> X("IdentifyLoopUnrolling", "Identify Loop Unrolling Potential.");
