/**
  ******************************************************************************
  * @file    rui_api.c
  * @brief   Radio User Interface main public API
  ******************************************************************************
**/

#pragma once

/**
 * Function to push instruction recieved from rUI host to the instruction queue. Must check if instruction
 * is valid and determine priority. Function needs to be short and optomized as it is
 * called in an ISR.
 *
 * @return	true if instruction was a valid rUI instruction, false otherwise
 */
bool RUI_InstructionPush(void)
{
	return false;
}

/**
 * Function to pop instruction recieved from rUI host from the instruction queue. Must check if instruction
 * is valid and determine priority. Function needs to be short and optomized as it is
 * called in an ISR.
 */
void RUI_InstructionPop(void)
{
	return;
}

/**
 * Function to peek instruction recieved from rUI host from the instruction queue. Must check if instruction
 * is valid and determine priority. Function needs to be short and optomized as it is
 * called in an ISR.
 */
void RUI_InstructionPeek(void)
{
	return;
}

/**
 * Function to peek the top of instruction recieved from rUI host from the instruction queue. Must check 
 * if instruction is valid and determine priority. Function needs to be short and optomized as it is
 * called in an ISR.
 */
void RUI_InstructionTop(void)
{
	return;
}