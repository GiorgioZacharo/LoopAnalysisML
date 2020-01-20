//===------------------------- IdentifyLoopUnrolling.h -------------------------===//
//
//                     The LLVM Compiler Infrastructure
// 
// This file is distributed under the Università della Svizzera italiana (USI) 
// Open Source License.
//
// Author         : Georgios Zacharopoulos 
// Date Started   : September, 2016
// Date Modified  : November, 2016
//===-------------------------------------------------------------------------===//
//
// This file identifies Loop Unrolling Potential in Regions of SW Applications.
//
//
// Computes Delay of a body of a loop unrolled by a given LOOP_UNROLLING_FACTOR 
//
//===-------------------------------------------------------------------------===//

#include "llvm/ADT/Statistic.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/CFG.h"
#include "llvm/Analysis/Interval.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Format.h"
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>

#ifndef LOOP_UNROLLING_FACTOR
#define LOOP_UNROLLING_FACTOR               1     // User Specified. (1 = No Loop Unrolling)
#endif

#ifndef NSECS_PER_CYCLE
#define NSECS_PER_CYCLE                    10      // nSecs Per Cycle
#endif

#define LOAD_LATENCY                       NSECS_PER_CYCLE * 7  // 7 Cycles -- 70  nSecs
#define STORE_LATENCY                      NSECS_PER_CYCLE * 5  // 5 Cycles -- 50  nSecs
#define SINGLE_LD_AND_ST_LATENCY           NSECS_PER_CYCLE     //  1 Cycle  -- 10  nSecs

std::ofstream DFGLUFile; // File that DFG graph is written.
std::ofstream FeatureVectorFile; // File that DFG graph is written.


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


unsigned int getLoopCarriedDependencies(BasicBlock *BB) {

  unsigned int LoopCarriedDep = 0;
  bool IndVariableFound = false;

  //DependenceAnalysis *DA = &getAnalysis<DependenceAnalysis>();

  for(BasicBlock::iterator BI = BB->begin(), BE = BB->end(); BI != BE; ++BI) {

    if (PHINode *PN  = dyn_cast<PHINode>(&*BI)) {

      for (unsigned int i =0; i < PN->getNumIncomingValues(); i++) {
        if (PN->getIncomingBlock(i) == BB) {
          LoopCarriedDep++;
          //errs() << "     Phi : " << *PN << "\n";
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

// Populate the Vectors with the Loop Carried Dependency Instructions. (Passing them by reference)
//
//
void getInstrWithLoopCarriedDependencies(BasicBlock *BB, std::vector<Instruction *> &LCD_Pred, std::vector<Instruction *> &LCD_Succ) {

  bool IndVariableFound = false;

  for(BasicBlock::iterator BI = BB->begin(), BE = BB->end(); BI != BE; ++BI) {

    if (PHINode *PN  = dyn_cast<PHINode>(&*BI)) {

      for (unsigned int i =0; i < PN->getNumIncomingValues(); i++) {
        if (PN->getIncomingBlock(i) == BB) {     
          // errs() << "     Phi : " << *PN << "\n";
          Value *DependencyVar         = PN->getIncomingValueForBlock(BB);
          Instruction *DependencyInstr = dyn_cast<Instruction>(DependencyVar);

          if (!IndVariableFound) {
            if (isInductionVariable(DependencyVar)){
              IndVariableFound = true;
              continue;
            }
          }

          // Populating the Vectors with the Loop Carried Dependency Instructions.
          // LCD_Pred.push_back(&*PN);
          // LCD_Succ.push_back(&*DependencyInstr);
          LCD_Pred.push_back(&*DependencyInstr);
          LCD_Succ.push_back(&*PN);

        }
      }
    }
  }

  // for (int i=0; i< LCD_Pred.size(); i++) {
  //     errs() << "   Predecessor LCD Nodes " << *LCD_Pred[i] << "\n"; // My debugging Info!
  //     errs() << "   Successors  LCD Nodes " << *LCD_Succ[i] << "\n"; // My debugging Info!  
  // }
}

int getInstOccurences(std::vector<Instruction *> list, Instruction *inst) {

  int NumberOfOccurences = 0;

  for (unsigned i = 0; i < list.size(); i++)
    if (list[i] == inst)
      NumberOfOccurences++;
  

  return NumberOfOccurences;
}

int isInstASuccessorToInstB(std::vector<Instruction *> listA, Instruction *instA, std::vector<Instruction *> listB, Instruction *instB ) {

  for (unsigned i = 0; i < listA.size(); i++){
    for (unsigned j = 0; j < listB.size(); j++)
      if ( i==j && listA[i] == instA && listB[j] ==instB )
        return i;
  }

  return -1;
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

int find_loop(std::vector<Loop *> loop_list, Loop *Loop) {

  for (unsigned i = 0; i < loop_list.size(); i++)
    if (loop_list[i] == Loop)
      return i;
  
  return -1;
}

int getNumberofLoads(BasicBlock *BB) {

  int NumberOfLoads = 0;

  for(BasicBlock::iterator BI = BB->begin(), BE = BB->end(); BI != BE; ++BI)
    if (dyn_cast<LoadInst>(&*BI))
      NumberOfLoads++;

  return NumberOfLoads;
}

int getNumberofStores(BasicBlock *BB) {

  int NumberOfStores = 0;

  for(BasicBlock::iterator BI = BB->begin(), BE = BB->end(); BI != BE; ++BI)
    if (dyn_cast<StoreInst>(&*BI))
      NumberOfStores++;

  return NumberOfStores;
}


// Data Dependence Analysis inside BB.
//
//
bool DataDependencyAnalysis(BasicBlock *BB, DependenceAnalysis *DA) {

  bool DataDep = false;

  std::vector<Instruction *> Loads, Stores;
  Loads.clear(), Stores.clear();

  for(BasicBlock::iterator BI = BB->begin(), BE = BB->end(); BI != BE; ++BI) {

    if (dyn_cast<StoreInst>(&*BI))
      Stores.push_back(&*BI);

    if (dyn_cast<LoadInst>(&*BI))
      Loads.push_back(&*BI);
  }

  // for (unsigned i = 0; i < Loads.size(); i++) {
  //   for (unsigned j = 0; j < Stores.size(); j++) {

  for (unsigned i = 0; i < Stores.size(); i++) {
    for (unsigned j = 0; j < Loads.size(); j++) {

      // errs() << i << " - " << j << " : " << *Loads[i] << "  -  " << *Stores[j] << "\n";
      if (j > i) { // Look only for future Loads from a given store.
      
        // if (std::unique_ptr<Dependence> Dep = DA->depends( Loads[i], Stores[j], false) ) {
        if (std::unique_ptr<Dependence> Dep = DA->depends( Stores[i], Loads[j], false) ) {

          errs() << " \n DD : "<< i << " " << j << " " << *Dep->getSrc() << " --> " << *Dep->getDst() << "\n";

          if(Dep->isFlow()) { 
            DataDep = true;
            errs() << " Flow Dependency! " << "\n";
          }

           if(Dep->isAnti())
            errs() << " Anti Dependency! " << "\n";
      

          if(Dep->isInput())
            errs() << " isInput Dependency! " << "\n";

          if(Dep->isOutput())
            errs() << " Output Dependency! " << "\n";

          if(Dep->isConfused())
            errs() << " Confused Dependency! " << "\n";

          if(Dep->isConsistent())
            errs() << " Consistent Dependency! " << "\n";
        

        }
    }
    
    }
  }
  errs() << "\n\n";
  return DataDep;
}

// Memory Delay for Loads or Stores inside BB. (Data Dependency across Loads and Stores Included)
//                                            
//
float getMemoryDelay(BasicBlock *BB, DependenceAnalysis *DA, int NumberOfMemoryAccesses, bool AccessesAreloads) {

  float MemoryDelay = 0;
  int MemoryLatency = 0;

  if (AccessesAreloads)
    MemoryLatency = LOAD_LATENCY;  // Case: Loads
  else
    MemoryLatency = STORE_LATENCY; // Case: Stores

  if(NumberOfMemoryAccesses) {

    if (NumberOfMemoryAccesses>1) {
      // Case: Data Dependencies.
      if (DataDependencyAnalysis(BB, DA))
        MemoryDelay = MemoryLatency * NumberOfMemoryAccesses;
      //Case: *No* Data Dependencies.
      else
        MemoryDelay = MemoryLatency + (NumberOfMemoryAccesses-1) * NSECS_PER_CYCLE;
    }

    else
      MemoryDelay = SINGLE_LD_AND_ST_LATENCY; // 1 Load or 1 STORE -> 1 Cycle
  }
  return MemoryDelay;
}


// Compute the Critical Path of HW Delay in Nsecs inside the body of an unrolled Loop.
// (With respect to the LOOP_UNROLLING_FACTOR provided.)
//
float getDelayOfUnrolledBB(BasicBlock *BB, DependenceAnalysis *DA) {

  float DelayOfBB = 0;
  std::vector<Instruction *> worklist, Predecessor_Nodes, Successor_Nodes, LCD_Pred, LCD_Succ, Pred_Nodes, Succ_Nodes;// Pred and Succ are permanent.
  std::vector<float> DelayPaths, DelayNodes;
  std::vector<int> IncomingValue, OutcomingValue; 

  worklist.clear(), Predecessor_Nodes.clear(), Successor_Nodes.clear(), LCD_Pred.clear(), LCD_Succ.clear(), Pred_Nodes, Succ_Nodes;
  DelayPaths.clear(), DelayNodes.clear();
  IncomingValue.clear(), OutcomingValue.clear();

  // Iterate inside the basic block and gather all DFG Nodes.
  for(BasicBlock::iterator BI = BB->begin(), BE = BB->end(); BI != BE; ++BI) {
    worklist.push_back(&*BI);
    DelayPaths.push_back(getDelayEstim(&*BI)); //Initialize the Delay Estimation for each DFG Node. - This will store the computed delay paths.
    DelayNodes.push_back(getDelayEstim(&*BI)); //Initialize the Delay Estimation for each DFG Node. - This will remain constant.
  }

  // Print the worklist.
  // errs() <<" Worklist \n";
  // for (int i=0; i< worklist.size(); i++)
  //     errs() << " DFG Node " << *worklist[i]  << "\n"; // My debugging Info!

  if (LOOP_UNROLLING_FACTOR >1)
    getInstrWithLoopCarriedDependencies(&*BB, LCD_Pred, LCD_Succ);

    //  for (int i=0; i< LCD_Pred.size(); i++) {
    //   errs() << "   Predecessor LCD Nodes " << *LCD_Pred[i] << "\n"; // My debugging Info!
    //   errs() << "   Successors  LCD Nodes " << *LCD_Succ[i] << "\n"; // My debugging Info!  
    // }   
  
  // Find Relation among DFG Nodes.
  //
  // Predecessor_Nodes --> Successor_Nodes
  //
  for (std::vector<Instruction *>::iterator iter = worklist.begin(); iter != worklist.end(); ++iter) {

    if(Instruction *Inst = *iter) {

      // Iterate over each operand of each Instruction.
      for (unsigned int i=0; i<Inst->getNumOperands(); i++) {

        Value *Operand = Inst->getOperand(i);

        if (PHINode *phi = dyn_cast<PHINode>(&*Inst) )
          if (phi->getIncomingBlock(i) != BB)
            continue;

        int count =0;  
        // Iterate over all the instructions of the Region and compare the operand to them.
        for (std::vector<Instruction *>::iterator instruction_iterator = worklist.begin(); instruction_iterator != worklist.end(); ++instruction_iterator, count++) {

          if(count <= find_inst(worklist, Inst) ) { // Might need to remove "=""

            if(Instruction * Inst_source = *instruction_iterator) {
              if (Operand == Inst_source) {

                Predecessor_Nodes.push_back(Inst_source); // Populate Predecessor_Nodes vector
                Successor_Nodes.push_back(Inst); // Populate Successor_Nodes vector
                Pred_Nodes.push_back(Inst_source); // Permanent addition
                Succ_Nodes.push_back(Inst); // Permanent addition
              }
            }
          }
        }
      }
    }
  }

    // // Print Predecessors/Successors list
    // errs() << "\n Preds/Succs \n" ;
    // for (int i=0; i< Predecessor_Nodes.size(); i++)
    //   errs() << " Edges " << *Predecessor_Nodes[i] << "  --->     " <<  *Successor_Nodes[i] << "\n"; // My debugging Info!

  if (Predecessor_Nodes.size() > 0 && Successor_Nodes.size() >0) {

    // Critical Path Estimation. 
    //       
    // 
    int instr_index = 0;
    for (std::vector<Instruction *>::iterator instr_iter = worklist.begin(); instr_iter != worklist.end(); ++instr_iter, instr_index++) {

      Instruction *instr_temp = *instr_iter;

      // Find the end Nodes (Bottom-most Nodes in the DFG Graph)
      if (find_inst(Predecessor_Nodes, instr_temp) == -1) {

        Instruction *EndNode = instr_temp;
        std::vector<Instruction *> BottomNodes;
        BottomNodes.clear();
        BottomNodes.push_back(EndNode);

        Instruction *CurrentNode;
        int position =0;

        //errs() << "\n\n CurrentNode  " << *EndNode << "\n";

        while(BottomNodes.size() >0) {  

          CurrentNode = BottomNodes[0];

          while (find_inst(Successor_Nodes, CurrentNode) >=0) {

            position = find_inst(Successor_Nodes, CurrentNode); // Position in Succesor List.
            
            int pred_position_worklist = find_inst(worklist, Predecessor_Nodes[position]); // Position of Pred in worklist.
            int cur_node_position_worklist = find_inst(worklist, CurrentNode); // Position of Current Node in worklist.
            Instruction *Predecessor = Predecessor_Nodes[position]; // Predecessor.

            errs() << "\n CPW " << cur_node_position_worklist;

            // Is Predecessor a Loop Carried Dependency successor?
            //
            if (find_inst(LCD_Succ, Predecessor) != -1) {



              int LCD_position = find_inst(LCD_Succ, Predecessor);

              // Do not take into account Sobelsystem like examples with double edges.
              if (isInstASuccessorToInstB( Succ_Nodes, Predecessor, Pred_Nodes, LCD_Pred[LCD_position]) == -1 ) {

                errs() << "   Pred  " << *Predecessor << "\t LCD pos " << LCD_position << "\t" ;

                // Is the LCD predecessor the current node?
                // Case A
                if (LCD_Pred[LCD_position] == CurrentNode ) { 
       
                  DelayPaths[pred_position_worklist] = std::max( DelayPaths[pred_position_worklist] , 
                    (DelayNodes[pred_position_worklist] +  DelayNodes[cur_node_position_worklist]) * LOOP_UNROLLING_FACTOR ); // Update the Delay Path of Predecessor

                  // DelayPaths[pred_position_worklist] = std::max( DelayPaths[pred_position_worklist] , 
                  //   DelayNodes[pred_position_worklist] +  DelayPaths[cur_node_position_worklist] ); // Update the Delay Path of Predecessor

                  // DelayPaths[pred_position_worklist] += (DelayNodes[pred_position_worklist] + DelayNodes[cur_node_position_worklist]) * 
                  //   (LOOP_UNROLLING_FACTOR -1);
                  //DelayPaths[pred_position_worklist] =  DelayPaths[pred_position_worklist] * (LOOP_UNROLLING_FACTOR -1);
                  errs()<< "A:  DelayPath of " << pred_position_worklist << " is : " << DelayPaths[pred_position_worklist]  << "\n";

                }

                // Is the LCD Predecessor, a Succesor of the Predecessor?
                // Case B
                else if (isInstASuccessorToInstB( Succ_Nodes, LCD_Pred[LCD_position], Pred_Nodes, Predecessor) != -1 ) {


                  int lcd_pred_position_worklist = find_inst(worklist, LCD_Pred[LCD_position]); // Worklist position of lcd predecessor.
                  
                  DelayPaths[pred_position_worklist] = std::max( DelayPaths[pred_position_worklist] , 
                    (DelayNodes[pred_position_worklist] +  DelayPaths[cur_node_position_worklist]) * (LOOP_UNROLLING_FACTOR) ); // Update the Delay Path of Predecessor

                  // Update the Delay Path of Predecessor (+= Predecessor Delay * (LUF -1) )
                  // DelayPaths[pred_position_worklist] +=  DelayPaths[pred_position_worklist] * (LOOP_UNROLLING_FACTOR -1);
                
                  errs()<< "B: DelayPath of " << pred_position_worklist << " is : " << DelayPaths[pred_position_worklist]  << "\n";

                }

                // Case C
                else {
                  int lcd_pred_position_worklist = find_inst(worklist, LCD_Pred[LCD_position]); // Worklist position of lcd predecessor.
                  
                  DelayPaths[pred_position_worklist] = std::max( DelayPaths[pred_position_worklist] , 
                    DelayNodes[pred_position_worklist] +  DelayPaths[cur_node_position_worklist] ); // Update the Delay Path of Predecessor

                  // Update the Delay Path of LCD Predecessor (Predecessor of Current Node * (LUF -1) )
                  DelayPaths[lcd_pred_position_worklist] = std::max( DelayPaths[lcd_pred_position_worklist] , 
                    DelayNodes[lcd_pred_position_worklist] +  (DelayPaths[pred_position_worklist] * (LOOP_UNROLLING_FACTOR -1)) );
                
                  errs()<< "C:  DelayPath of " << pred_position_worklist << " is : " << DelayPaths[pred_position_worklist]  << "\n";
                  errs()<< "C:  DelayPath of " << lcd_pred_position_worklist << " is : " << DelayPaths[lcd_pred_position_worklist]  << "\n";
                }
              }
            } //End of LCD check if

            // Normal Delay Path Update to find Critical Path.
            else {
              DelayPaths[pred_position_worklist] = std::max( DelayPaths[pred_position_worklist] , 
                DelayNodes[pred_position_worklist] +  DelayPaths[cur_node_position_worklist] );
            }
           
            //errs() << "   Pred  " << *Predecessor << "\t" ;
            //errs() << "   Del Path  " <<DelayPaths[pred_position_worklist] << "\n";

            Successor_Nodes.erase(Successor_Nodes.begin() + position);            // deleting the last edge
            Predecessor_Nodes.erase(Predecessor_Nodes.begin() + position);        //  deleting the last edge


            if (find_inst(Predecessor_Nodes, Predecessor) == -1) // Is predecessor Bottom-most now?
              BottomNodes.push_back(Predecessor);

          }

          BottomNodes.erase(BottomNodes.begin()); // Delete the First element of the BottomNodes list.
        } // End of while.
      } // End of if.

    } // End of for.

    // Debug!! ***   Latest!    ***
    // errs()<< "\n\n";
    // for (unsigned i = 0; i < DelayPaths.size(); i++){
    //   errs()<< "  DelayPath " << i << " : " << DelayPaths[i]  << "\t";
    //   errs()<< "  DelayNode " << i << " : " << DelayNodes[i]  << "\n";
    // }

    DelayOfBB = get_max(DelayPaths); // Get the Final (maximum) Value of Computational Delay within a BB.
  }

  else
    for (unsigned i = 0; i < worklist.size(); i++)
      DelayOfBB += getDelayEstim(worklist[i]);

  //===----------------------------------------------------------------------===//
  // Compute Memory Delay.
  //===----------------------------------------------------------------------===//
  //
  // Get Loads and Stores in the BB.
  //

  int NumberOfLoads = getNumberofLoads(BB);
  int NumberOfStores = getNumberofStores(BB);
  float LoadsDelay = 0, StoresDelay = 0, LoadsAndStoresDelay = 0;

  LoadsDelay  = getMemoryDelay(BB, DA, NumberOfLoads, true);
  StoresDelay = getMemoryDelay(BB, DA, NumberOfStores, false); 

  // Sum up the Loads and Stores memory Delay.
  LoadsAndStoresDelay = LoadsDelay + StoresDelay;

  // Account for the Loop Unrolling factor.
  LoadsAndStoresDelay *= LOOP_UNROLLING_FACTOR; 

  /* Deactivated for now */

 // Compare Memory to Computation Delay.
 // if (LoadsAndStoresDelay > DelayOfBB)
 //  DelayOfBB = LoadsAndStoresDelay;

  errs() << "\n Loads/Stores Delay Estimation for BB is : " << format("%.8f", LoadsAndStoresDelay) << "\n";
  errs() << " Loads Number: " << NumberOfLoads << "\n" << " Stores Number: " << NumberOfStores<< "\n";
  //errs() << " Delay Estimation for BB is : " << format("%.8f", DelayOfBB) << "\n";
    
  // for (int i=0; i< DelayPaths.size(); i++)
  //   errs() << " Delays " << format("%.8f", DelayPaths[i]) << "\n"; // My debugging Info!

  return DelayOfBB;
}


  // Print the Data Flow Graph of the Unrolled BB.
  //
  //
  void DFGPrinterLoopUnrolledBB(BasicBlock *BB) {

    std::vector<Instruction *> worklist, predecessor_node, successor_node, LCD_Pred, LCD_Succ;
    std::vector<std::string> Color;
    int instr_count = 0;

    // Clear Vectors.
    worklist.clear(), predecessor_node.clear(), successor_node.clear(), LCD_Pred.clear(), LCD_Succ.clear(), Color.clear();

    // Populate the Vectors with the Loop Carried Dependency Instructions.
    getInstrWithLoopCarriedDependencies(&*BB, LCD_Pred, LCD_Succ);

      // for (int i=0; i< LCD_Pred.size(); i++) {
      //   errs() << "   Predecessor LCD Nodes " << *LCD_Pred[i] << "\n"; // My debugging Info!
      //   errs() << "   Successors  LCD Nodes " << *LCD_Succ[i] << "\n"; // My debugging Info!  
      // }   

    std::string FuncName = BB->getParent()->getName();
    std::string BBName = BB->getName();

    DFGLUFile.open ("DFG_" + FuncName + "_" + BBName + ".gv", std::ofstream::out | std::ofstream::app); 
    DFGLUFile << "digraph \"" << FuncName << "_" << BBName << "\" {" << "\n";
    DFGLUFile.close();

    // Iterate inside the basic block and gather all DFG Nodes.
    for(BasicBlock::iterator BI = BB->begin(), BE = BB->end(); BI != BE; ++BI) {
      worklist.push_back(&*BI);
    }

      
    for (std::vector<Instruction *>::iterator iter = worklist.begin(); iter != worklist.end(); ++iter, instr_count++) {

      Instruction *Instr = *iter;
      std::string Instr_Name = Instr->getOpcodeName();

      DFGLUFile.open ("DFG_" + FuncName + "_" + BBName + ".gv", std::ofstream::out | std::ofstream::app); 
      DFGLUFile << "N" << instr_count << "_" <<  Instr_Name << " [weight = 1, style = filled]\n";
      DFGLUFile.close();
    }

    
    // Find Relation among DFG Nodes.
    //
    // predecessor_Node --> successor_Node
    //
    for (std::vector<Instruction *>::iterator iter = worklist.begin(); iter != worklist.end(); ++iter) {

      if(Instruction *Inst = *iter) {

        // Iterate over each operand of each Instruction.
        for (unsigned int i=0; i<Inst->getNumOperands(); i++) {

          Value *Operand = Inst->getOperand(i);

          if (PHINode *phi = dyn_cast<PHINode>(&*Inst) )
            if (phi->getIncomingBlock(i) != BB)
              continue;

          int count =0;  
          // Iterate over all the instructions of the Region and compare the operand to them.
          for (std::vector<Instruction *>::iterator instruction_iterator = worklist.begin(); 
                instruction_iterator != worklist.end(); ++instruction_iterator, count++) {

            if(count <= find_inst(worklist, Inst) ) { // Might need to remove "=""

              if(Instruction * Inst_source = *instruction_iterator) {
                if (Operand == Inst_source) {

                  predecessor_node.push_back(Inst_source); // Populate predecessor_node vector
                  successor_node.push_back(Inst); // Populate successor_node vector
                  Color.push_back("[color = black]");
                }
              }
            }
          }
        }
      }
    }

    // Print The LCD Edges.
    for (int i=0; i< LCD_Pred.size(); i++) {
      predecessor_node.push_back(LCD_Pred[i]);  
      successor_node.push_back(LCD_Succ[i]);
      Color.push_back("[color = red]");
    }

    for (int i=0; i< predecessor_node.size(); i++) {

      int pred_pos = find_inst(worklist, predecessor_node[i]);
      int succ_pos = find_inst(worklist, successor_node[i]);

      DFGLUFile.open ("DFG_" + FuncName + "_" + BBName + ".gv", std::ofstream::out | std::ofstream::app); 
      DFGLUFile << "N" << pred_pos << "_" << predecessor_node[i]->getOpcodeName() << " -> " << "N" << succ_pos << "_" << successor_node[i]->getOpcodeName() <<
            Color[i] <<" ;\n";
      DFGLUFile.close();

    }

    DFGLUFile.open ("DFG_" + FuncName + "_" + BBName + ".gv", std::ofstream::out | std::ofstream::app); 
    DFGLUFile << "}";
    DFGLUFile.close();

  }
