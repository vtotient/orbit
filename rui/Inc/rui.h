/**
  ******************************************************************************
  * @file    rui.h
  * @brief   Radio User Interface main header file
  ******************************************************************************
**/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define RUI_INSTRUCTION_SIZE_BYTES	4 	// Size of an rUI instruction
#define RUI_INSTRUCTION_QUEUE_DEPTH	10 	// Depth of instruction queue

/* Instruction queue struct */
typedef union
{
	uint32_t queueEntry;

	struct
	{
		uint8_t header 		: 2;
		uint8_t extend 		: 1;
		uint8_t priority 	: 1;
		uint8_t instruction : 28;
	} fields;
} RUI_InstructionQueueTypedef;

/* Instruction queue functions */
bool RUI_InstructionPush(void);
void RUI_InstructionPop(void);
void RUI_InstructionPeek(void);
void RUI_InstructionTop(void);

/* Instruction queue */
RUI_InstructionQueueTypedef RUI_InstructionQueue[RUI_INSTRUCTION_QUEUE_DEPTH];