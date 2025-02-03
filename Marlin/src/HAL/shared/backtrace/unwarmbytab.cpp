/**
 * Libbacktrace
 * Copyright 2015 Stephen Street <stephen@redrocketcomputing.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://www.mozilla.org/en-US/MPL/2.0/
 *
 * This library was modified, some bugs fixed, stack address validated
 * and adapted to be used in Marlin 3D printer firmware as backtracer
 * for exceptions for debugging purposes in 2018 by Eduardo José Tagle.
 */

#if defined(__arm__) || defined(__thumb__)

#include "unwarmbytab.h"

#include <stdint.h>
#include <string.h>

/* These symbols point to the unwind index and should be provide by the linker script */
extern "C" const UnwTabEntry __exidx_start[];
extern "C" const UnwTabEntry __exidx_end[];

/* This prevents the linking of libgcc unwinder code */
void __aeabi_unwind_cpp_pr0() {};
void __aeabi_unwind_cpp_pr1() {};
void __aeabi_unwind_cpp_pr2() {};

static inline __attribute__((always_inline)) uint32_t prel31_to_addr(const uint32_t *prel31) {
  uint32_t offset = (((uint32_t)(*prel31)) << 1) >> 1;
  return ((uint32_t)prel31 + offset) & 0x7FFFFFFF;
}

static const UnwTabEntry *UnwTabSearchIndex(const UnwTabEntry *start, const UnwTabEntry *end, uint32_t ip) {
  const UnwTabEntry *middle;

  /* Perform a binary search of the unwind index */
  while (start < end - 1) {
    middle = start + ((end - start + 1) >> 1);
    if (ip < prel31_to_addr(&middle->addr_offset))
      end = middle;
    else
      start = middle;
  }
  return start;
}

/**
 * Get the function name or nullptr if not found
 */
static const char *UnwTabGetFunctionName(const UnwindCallbacks *cb, uint32_t address) {
  uint32_t flag_word = 0;
  if (!cb->readW(address-4,&flag_word))
    return nullptr;

  if ((flag_word & 0xFF000000) == 0xFF000000) {
    const uint32_t fn_name_addr = address - 4 - (flag_word & 0x00FFFFFF);

    // Ensure the address is readable to avoid returning a bogus pointer
    uint8_t dummy = 0;
    if (cb->readB(fn_name_addr, &dummy))
      return (const char *)fn_name_addr;
  }
  return nullptr;
}

/**
 * Get the next frame unwinding instruction
 *
 * Return either the instruction or -1 to signal no more instructions
 * are available
 */
static int UnwTabGetNextInstruction(const UnwindCallbacks *cb, UnwTabState *ucb) {
  int instruction;

  /* Are there more instructions */
  if (ucb->remaining == 0)
    return -1;

  /* Extract the current instruction */
  uint32_t v = 0;
  if (!cb->readW(ucb->current, &v))
    return -1;
  instruction = (v >> (ucb->byte << 3)) & 0xFF;

  /* Move the next byte */
  --ucb->byte;
  if (ucb->byte < 0) {
    ucb->current += 4;
    ucb->byte = 3;
  }
  --ucb->remaining;

  return instruction;
}

/**
 * Initialize the frame unwinding state
 */
static UnwResult UnwTabStateInit(const UnwindCallbacks *cb, UnwTabState *ucb, uint32_t instructions, const UnwindFrame *frame) {

  /* Initialize control block */
  memset(ucb, 0, sizeof(UnwTabState));
  ucb->current = instructions;

  /* Is a short unwind description */
  uint32_t v = 0;
  if (!cb->readW(instructions, &v))
    return UNWIND_DREAD_W_FAIL;

  if ((v & 0xFF000000) == 0x80000000) {
    ucb->remaining = 3;
    ucb->byte = 2;
  /* Is a long unwind description */
  } else if ((v & 0xFF000000) == 0x81000000) {
    ucb->remaining = ((v & 0x00FF0000) >> 14) + 2;
    ucb->byte = 1;
  } else
    return UNWIND_UNSUPPORTED_DWARF_PERSONALITY;

  /* Initialize the virtual register set */
  ucb->vrs[7] = frame->fp;
  ucb->vrs[13] = frame->sp;
  ucb->vrs[14] = frame->lr;
  ucb->vrs[15] = 0;

  /* All good */
  return UNWIND_SUCCESS;
}

/**
 * Execute unwinding instructions
 */
static UnwResult UnwTabExecuteInstructions(const UnwindCallbacks *cb, UnwTabState *ucb) {
  int instruction;
  uint32_t mask, reg, vsp;

  /* Consume all instruction byte */
  while ((instruction = UnwTabGetNextInstruction(cb, ucb)) != -1) {

    if ((instruction & 0xC0) == 0x00) { // ARM_EXIDX_CMD_DATA_POP
      /* vsp += (xxxxxx << 2) + 4 */
      ucb->vrs[13] += ((instruction & 0x3F) << 2) + 4;
    }
    else if ((instruction & 0xC0) == 0x40) { // ARM_EXIDX_CMD_DATA_PUSH
      /* vsp -= (xxxxxx << 2) - 4 */
      ucb->vrs[13] -= ((instruction & 0x3F) << 2) - 4;
    }
    else if ((instruction & 0xF0) == 0x80) {
      /* pop under mask {r15-r12},{r11-r4} or refuse to unwind */
      instruction = instruction << 8 | UnwTabGetNextInstruction(cb, ucb);

      /* Check for refuse to unwind */
      if (instruction == 0x8000)        // ARM_EXIDX_CMD_REFUSED
        return UNWIND_REFUSED;

      /* Pop registers using mask */    // ARM_EXIDX_CMD_REG_POP
      vsp = ucb->vrs[13];
      mask = instruction & 0xFFF;

      reg = 4;
      while (mask) {
        if ((mask & 1) != 0) {
          uint32_t v;
          if (!cb->readW(vsp,&v))
            return UNWIND_DREAD_W_FAIL;
          ucb->vrs[reg] = v;
          v += 4;
        }
        mask >>= 1;
        ++reg;
      }

      /* Patch up the vrs sp if it was in the mask */
      if (instruction & (1 << (13 - 4)))
        ucb->vrs[13] = vsp;
    }
    else if ((instruction & 0xF0) == 0x90 // ARM_EXIDX_CMD_REG_TO_SP
           && instruction != 0x9D
           && instruction != 0x9F
    ) {
      /* vsp = r[nnnn] */
      ucb->vrs[13] = ucb->vrs[instruction & 0x0F];
    }
    else if ((instruction & 0xF0) == 0xA0) { // ARM_EXIDX_CMD_REG_POP
      /* pop r4-r[4+nnn] or pop r4-r[4+nnn], r14*/
      vsp = ucb->vrs[13];

      for (reg = 4; reg <= uint32_t((instruction & 0x07) + 4); ++reg) {
        uint32_t v;
        if (!cb->readW(vsp,&v))
          return UNWIND_DREAD_W_FAIL;

        ucb->vrs[reg] = v;
        vsp += 4;
      }

      if (instruction & 0x08) { // ARM_EXIDX_CMD_REG_POP
        uint32_t v;
        if (!cb->readW(vsp,&v))
          return UNWIND_DREAD_W_FAIL;
        ucb->vrs[14] = v;
        vsp += 4;
      }

      ucb->vrs[13] = vsp;

    }
    else if (instruction == 0xB0) { // ARM_EXIDX_CMD_FINISH
      /* finished */
      if (ucb->vrs[15] == 0)
        ucb->vrs[15] = ucb->vrs[14];

      /* All done unwinding */
      return UNWIND_SUCCESS;

    }
    else if (instruction == 0xB1) { // ARM_EXIDX_CMD_REG_POP
      /* pop register under mask {r3,r2,r1,r0} */
      vsp = ucb->vrs[13];
      mask = UnwTabGetNextInstruction(cb, ucb);

      reg = 0;
      while (mask) {
        if ((mask & 1) != 0) {
          uint32_t v;
          if (!cb->readW(vsp,&v))
            return UNWIND_DREAD_W_FAIL;

          ucb->vrs[reg] = v;
          vsp += 4;
        }
        mask >>= 1;
        ++reg;
      }
      ucb->vrs[13] = (uint32_t)vsp;

    }
    else if (instruction == 0xB2) { // ARM_EXIDX_CMD_DATA_POP
      /* vps = vsp + 0x204 + (uleb128 << 2) */
      ucb->vrs[13] += 0x204 + (UnwTabGetNextInstruction(cb, ucb) << 2);
    }
    else if (instruction == 0xB3 // ARM_EXIDX_CMD_VFP_POP
          || instruction == 0xC8
          || instruction == 0xC9
    ) {
      /* pop VFP double-precision registers */
      vsp = ucb->vrs[13];

      /* D[ssss]-D[ssss+cccc] */
      uint32_t v;
      if (!cb->readW(vsp,&v))
        return UNWIND_DREAD_W_FAIL;

      ucb->vrs[14] = v;
      vsp += 4;

      if (instruction == 0xC8) {
        /* D[16+sssss]-D[16+ssss+cccc] */
        ucb->vrs[14] |= 1 << 16;
      }

      if (instruction != 0xB3) {
        /* D[sssss]-D[ssss+cccc] */
        ucb->vrs[14] |= 1 << 17;
      }

      ucb->vrs[13] = vsp;
    }
    else if ((instruction & 0xF8) == 0xB8 || (instruction & 0xF8) == 0xD0) {
      /* Pop VFP double precision registers D[8]-D[8+nnn] */
      ucb->vrs[14] = 0x80 | (instruction & 0x07);
      if ((instruction & 0xF8) == 0xD0)
        ucb->vrs[14] = 1 << 17;
    }
    else
      return UNWIND_UNSUPPORTED_DWARF_INSTR;
  }
  return UNWIND_SUCCESS;
}

static inline __attribute__((always_inline)) uint32_t read_psp() {
  /* Read the current PSP and return its value as a pointer */
  uint32_t psp;

  __asm__ volatile (
    "   mrs %0, psp \n"
    : "=r" (psp) : :
  );

  return psp;
}

/**
 * Unwind the specified frame and goto the previous one
 */
static UnwResult UnwTabUnwindFrame(const UnwindCallbacks *cb, UnwindFrame *frame) {

  UnwResult err;
  UnwTabState ucb;
  const UnwTabEntry *index;
  uint32_t instructions;

  /* Search the unwind index for the matching unwind table */
  index = UnwTabSearchIndex(__exidx_start, __exidx_end, frame->pc);

  /* Make sure we can unwind this frame */
  if (index->insn == 0x00000001)
    return UNWIND_SUCCESS;

  /* Get the pointer to the first unwind instruction */
  if (index->insn & 0x80000000)
    instructions = (uint32_t)&index->insn;
  else
    instructions = prel31_to_addr(&index->insn);

  /* Initialize the unwind control block */
  if ((err = UnwTabStateInit(cb, &ucb, instructions, frame)) < 0)
    return err;

  /* Execute the unwind instructions */
  err = UnwTabExecuteInstructions(cb, &ucb);
  if (err < 0)
    return err;

  /* Set the virtual pc to the virtual lr if this is the first unwind */
  if (ucb.vrs[15] == 0)
    ucb.vrs[15] = ucb.vrs[14];

  /* Check for exception return */
  /* TODO Test with other ARM processors to verify this method. */
  if ((ucb.vrs[15] & 0xF0000000) == 0xF0000000) {
    /* According to the Cortex Programming Manual (p.44), the stack address is always 8-byte aligned (Cortex-M7).
       Depending on where the exception came from (MSP or PSP), we need the right SP value to work with.

       ucb.vrs[7] contains the right value, so take it and align it by 8 bytes, store it as the current
       SP to work with (ucb.vrs[13]) which is then saved as the current (virtual) frame's SP.
    */
    uint32_t stack;
    ucb.vrs[13] = (ucb.vrs[7] & ~7);

    /* If we need to start from the MSP, we need to go down X words to find the PC, where:
        X=2  if it was a non-floating-point exception
        X=20 if it was a floating-point (VFP) exception

       If we need to start from the PSP, we need to go up exactly 6 words to find the PC.
       See the ARMv7-M Architecture Reference Manual p.594 and Cortex-M7 Processor Programming Manual p.44/p.45 for details.
    */
    if ((ucb.vrs[15] & 0xC) == 0) {
      /* Return to Handler Mode: MSP (0xFFFFFF-1) */
      stack = ucb.vrs[13];

      /* The PC is always 2 words down from the MSP, if it was a non-floating-point exception */
      stack -= 2*4;

      /* If there was a VFP exception (0xFFFFFFE1), the PC is located another 18 words down */
      if ((ucb.vrs[15] & 0xF0) == 0xE0) {
        stack -= 18*4;
      }
    }
    else {
      /* Return to Thread Mode: PSP (0xFFFFFF-d) */
      stack = read_psp();

      /* The PC is always 6 words up from the PSP */
      stack += 6*4;
    }

    /* Store the PC */
    uint32_t v;
    if (!cb->readW(stack,&v))
      return UNWIND_DREAD_W_FAIL;
    ucb.vrs[15] = v;
    stack -= 4;

    /* Store the LR */
    if (!cb->readW(stack,&v))
      return UNWIND_DREAD_W_FAIL;
    ucb.vrs[14] = v;
    stack -= 4;
  }

  /* We are done if current frame pc is equal to the virtual pc, prevent infinite loop */
  if (frame->pc == ucb.vrs[15])
    return UNWIND_SUCCESS;

  /* Update the frame */
  frame->fp = ucb.vrs[7];
  frame->sp = ucb.vrs[13];
  frame->lr = ucb.vrs[14];
  frame->pc = ucb.vrs[15];

  /* All good - Continue unwinding */
  return UNWIND_MORE_AVAILABLE;
}

UnwResult UnwindByTableStart(UnwindFrame* frame, const UnwindCallbacks *cb, void *data) {

  UnwResult err = UNWIND_SUCCESS;
  UnwReport entry;

  /* Use DWARF unwind information to unwind frames */
  do {
    if (frame->pc == 0) {
      /* Reached __exidx_end. */
      break;
    }

    if (frame->pc == 0x00000001) {
      /* Reached .cantunwind instruction. */
      break;
    }

    /* Find the unwind index of the current frame pc */
    const UnwTabEntry *index = UnwTabSearchIndex(__exidx_start, __exidx_end, frame->pc);

    /* Clear last bit (Thumb indicator) */
    frame->pc &= 0xFFFFFFFEU;

    /* Generate the backtrace information */
    entry.address = frame->pc;
    entry.function = prel31_to_addr(&index->addr_offset);
    entry.name = UnwTabGetFunctionName(cb, entry.function);
    if (!cb->report(data,&entry))
      break;

    /* Unwind frame and repeat */
  } while ((err = UnwTabUnwindFrame(cb, frame)) == UNWIND_MORE_AVAILABLE);

  /* All done */
  return err;
}

#endif // __arm__ || __thumb__
