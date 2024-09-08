#ifndef _GB_H_
#define _GB_H_ 1

#define ENDIAN 0 // 0:little-endian 1:big-endian 

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#define IASSERT assert 

#define _inout_
#define _in_
#define _out_

#ifndef nullptr
# ifdef __cplusplus
#  define nullptr    0
# else
#  define nullptr    ((void *)0)
# endif
#endif

extern  uint32_t dbg_flags;

#ifdef _MSC_VER
# define IR_ALIGN(x) __declspec (align (x))
# define IR_FORCEINLINE __forceinline
# define IR_STATIC_FORCEINLINE static __forceinline
# define IR_STATIC_NOINLINE static __declspec (noinline)
# define IR_INLINE inline
# define IR_NOINLINE __declspec (noinline)
# define IR_NO_VTABLE __declspec (novtable)
# define IR_SELECTANY __declspec(selectany)
# define IR_NODEFAULT() __assume (0)
# define IR_UNREACHED(expr) __assume (expr)
# define IR_FASTCALL __fastcall
# define IR_LIKELY(expr) (expr)
# define IR_UNLIKELY(expr) (expr)

# ifdef _WIN32
#  ifdef _MSC_VER
#    define IR_FAST_X86_SIMD_VECTOR
#  endif 

#  include <Windows.h>
#  include <intrin.h>
# endif 
#else 
# define IR_ALIGN(x)  __attribute__ ((aligned(x)))
# define IR_FORCEINLINE __attribute__((always_inline))
# define IR_STATIC_NOINLINE static __attribute__((noinline))
# define IR_STATIC_FORCEINLINE static __attribute__((always_inline))
# define IR_INLINE inline
# define IR_NOINLINE __attribute__((noinline))
# define IR_NO_VTABLE 
# define IR_LIKELY(expr) (expr)
# define IR_UNLIKELY(expr) (expr)
# define IR_NODEFAULT()
# define IR_UNREACHED(expr) __assume (expr)
# define IR_FASTCALL
#endif 

#define INT_VBL 0x01
#define INT_STAT 0x02 
#define INT_TIMER 0x04
#define INT_SERIAL 0x08 
#define INT_JOYPAD 0x10

#define MEM_1K 1024
#define MEM_2K 2048
#define MEM_4K 4096 
#define MEM_8K 8192 
#define MEM_16K 16384
#define MEM_32K 32768
#define MEM_64K 65536
#define MEM_1MB 1048576

// get_type 
// T


struct nbyte
{ // trace byte 
  uint8_t memory;
  uint8_t trace; // d0:exec breakpoint (fast) 
                 // d1:exec breakpoint (cond) 
                 // d2:memread breakpoint (fast) 
                 // d3:memread breakpoint (cond) 
                 // -------------------------------------------------
                 // d4:memwrite breakpoint (fast) 
                 // d5:memwrite breakpoint (cond) 
                 // d7_d6:
                 //      0 <- normal 
                 //      1 <- step-over 
                 //      2 <- cursor breakpoint 
                 //      3 <- step-out 
                 
};

#if IRESIC_ENDIAN == 0
# define REGISTER_BLOCK(lname, hname) \
    uint8_t lname;         \
    uint8_t hname;
#elif IRESIC_ENDIAN == 1
# define REGISTER_BLOCK(name) \
    uint8_t hname;         \
    uint8_t lname;        
#endif 

#define DEFINE_REGISTER(name, hname, lname) \
  union                       \
  {                           \
    struct                    \
    {                         \
      REGISTER_BLOCK(lname, hname)    \
    };                        \
    uint16_t name;            \
  };

template <class T>
struct mpu
{
  DEFINE_REGISTER(af, a, f)
  DEFINE_REGISTER(bc, b, c)
  DEFINE_REGISTER(de, d, e)
  DEFINE_REGISTER(hl, h, l)
  DEFINE_REGISTER(sp, sph, spl)
  DEFINE_REGISTER(pc, pch, pcl)

  struct ir_chunk
  { // invoke record chunk 
     uint16_t address; // address record
      uintptr_t id; // bank id
  };

  struct invoke_chunk
  {
    ir_chunk to; // call address 
    ir_chunk next; // ret address 
    ir_chunk cur; // cur address, 0xffff for interrupt 
    ir_chunk unused;
  };

  struct invoke_collector
  {
    invoke_chunk *stack;

    uintptr_t elements;
    uintptr_t maximum; // cache
  };

  struct addr_map
  { // address mapper segment 
    nbyte *ptr; // memory pointer 
      uintptr_t id; // bank id 
  };
  uint32_t zf; // program status byte flag decode 
  uint32_t nf; // program status byte flag decode 
  uint32_t hf; // program status byte flag decode 
  uint32_t cf; // program status byte flag decode 

  uint32_t ie_mask;
  uint32_t if_mask;
  uint32_t ime_mask;
  uint32_t int_pending; // d4~d0: int mask combine mask  
                        // d5:halt state 
                        // d6:frame reach mask 
                        // d7:skip one instruction interrupt mask 
                        // d8:single-step command mask 
  uint8_t ss_exec_bp_rmask; // no single-step ? 0xc3 : 0 
  uint8_t ss_rd_bp_rmask; // no single-step ? 0x0c : 0 
  uint8_t ss_wr_bp_rmask; // no single-step ? 0x30 : 0 
  uint8_t ss_acc_bp_rmask; // no single-step ? 0x3c : 0 
  uint16_t ss_halt_rmask; // no halt ? 0x0100 : 0 

  uint16_t halt_int_rmask; // halt ? 0x1f : 0 

  addr_map mmu[16]; // memory bank pointer 
                    // 
                    //  mem[1] breakpoint mask 
                    // 
                    //  d0:exec breakpoint (fast) 
                    //  d1:exec breakpoint (cond) 
                    //  d2:memread breakpoint (fast) 
                    //  d3:memread breakpoint (cond) 
                    //  -------------------------------------------------
                    //  d4:memwrite breakpoint (fast) 
                    //  d5:memwrite breakpoint (cond) 
                    //  d7_d6:
                    //       0 <- normal 
                    //       1 <- step-over 
                    //       2 <- cursor breakpoint 
                    //       3 <- step-out 

  int32_t spin_ticks;

  invoke_collector invoke_collector;

  nbyte wram[MEM_4K * 8];
  nbyte io[512];

  uint8_t key1; 
  uint8_t svbk;

  uint8_t opcodes[4];
  uint32_t speed_switch; // 1 for double speed, 0 for normal speed 

  uint16_t gbp_mask; // global breakpoint mask */
  uint16_t gbp[4];

  uint16_t m_read_addr;
  uint16_t m_write_addr;
  uint16_t m_write_val;

  uint32_t bp_type;// 0:read 8 bit 
                   // 1:read 16 bit 
                   // 2:exec 
                   // 3:state 
                   // 4:write 8 bit 
                   // 5:write 16 bit 

  uint8_t *temp_bp; // step over, cursor or step out break
  uint32_t temp_bp_confirm;

  IR_FORCEINLINE
  void interrupt_requset ( uint8_t mask, uint32_t mask2 )
  {
    mpu::if_mask |= (mask & mask2);
     
    mpu::int_pending &= ~0x1f;
    mpu::int_pending |= mpu::ie_mask 
                    & mpu::if_mask 
                      & mpu::ime_mask
                    & 0x1f;
  }

  IR_FORCEINLINE
  void interrupt_update (void)
  {
    mpu::int_pending &= ~0x1f;
    mpu::int_pending |= mpu::ie_mask 
                    & mpu::if_mask 
                      & mpu::ime_mask
                    & 0x1f;
  }

  intptr_t 
  bp_expr_rt (uintptr_t address )
  {
    return 1;
  }

  IR_FORCEINLINE
  uint8_t read_directly (uint16_t address) const 
  {
    return mpu::mmu[address >> 12].ptr[address & 0xfff].memory;
  }

  IR_FORCEINLINE
  void write_directly (uint16_t address, uint8_t value) 
  {
    mpu::mmu[address >> 12].ptr[address & 0xfff].memory = value;
  }

  IR_FORCEINLINE
  nbyte &access_directly (uint16_t address, uint8_t value) 
  {
    mpu::mmu[address >> 12].ptr[address & 0xfff];
  }

  IR_FORCEINLINE void 
  flag2_regf (void)
  {
    mpu::f &= 0x0f;
    mpu::f |= (mpu::zf == 0) ? 0x80 : 0x00;
    mpu::f |= (mpu::nf != 0) ? 0x40 : 0x00;
    mpu::f |= (mpu::hf & 0x10) ? 0x20 : 0x00;
    mpu::f |= (mpu::cf & 0x100) ? 0x10 : 0x00;
  }

  IR_FORCEINLINE void 
  regf2_flag (void)
  {
    mpu::zf = (mpu::f & 0x80) ? 0 : UINT32_MAX;
    mpu::nf = (mpu::f & 0x40) ? UINT32_MAX : 0;
    mpu::hf = (mpu::f & 0x20) ? UINT32_MAX : 0;
    mpu::cf = (mpu::f & 0x10) ? UINT32_MAX : 0;
  }

  int init (void)
  {
    return 0;
  }

  int reset (void)
  {
    T *gb = static_cast<T *>(this);

    cart<T> &cart_ = * gb;
    ppu<T> &gpu_ = * gb;

    if ( gb->gb::get_type () == 0 )
    {
      mpu::af = 0x01b0;
      mpu::bc = 0x0013;
      mpu::de = 0x00d8;
      mpu::hl = 0x014d;
      mpu::sp = 0xfffe;
      mpu::pc = 0x0100;

      mpu::regf2_flag ();
    }
    else
    {
      mpu::af = 0x1180;
      mpu::bc = 0x0000;
      mpu::de = 0xff56;
      mpu::hl = 0x000d;
      mpu::sp = 0xfffe;
      mpu::pc = 0x0100;

      mpu::regf2_flag ();
    }
    mpu::mmu[0].ptr = & cart_.prom[MEM_4K * 0];
    mpu::mmu[1].ptr = & cart_.prom[MEM_4K * 1];
    mpu::mmu[2].ptr = & cart_.prom[MEM_4K * 2];
    mpu::mmu[3].ptr = & cart_.prom[MEM_4K * 3];
    mpu::mmu[4].ptr = & cart_.prom[MEM_4K * 4];
    mpu::mmu[5].ptr = & cart_.prom[MEM_4K * 5];
    mpu::mmu[6].ptr = & cart_.prom[MEM_4K * 6];
    mpu::mmu[7].ptr = & cart_.prom[MEM_4K * 7];
    mpu::mmu[8].ptr = & gpu_.vram[MEM_4K * 0];
    mpu::mmu[9].ptr = & gpu_.vram[MEM_4K * 1];
    mpu::mmu[10].ptr = & cart_.sram[MEM_4K * 0];
    mpu::mmu[11].ptr = & cart_.sram[MEM_4K * 1];
    mpu::mmu[12].ptr = & mpu::wram[MEM_4K * 0];
    mpu::mmu[13].ptr = & mpu::wram[MEM_4K * 1];
    mpu::mmu[14].ptr = & mpu::wram[MEM_4K * 0];
    mpu::mmu[15].ptr = & mpu::wram[MEM_4K * 1];

    mpu::mmu[0].id = 0;
    mpu::mmu[1].id = 0;
    mpu::mmu[2].id = 0;
    mpu::mmu[3].id = 0;
    mpu::mmu[4].id = 1;
    mpu::mmu[5].id = 1;
    mpu::mmu[6].id = 1;
    mpu::mmu[7].id = 1;
    mpu::mmu[8].id = 0;
    mpu::mmu[9].id = 0;
    mpu::mmu[10].id = 0;
    mpu::mmu[11].id = 0;
    mpu::mmu[12].id = 0;
    mpu::mmu[13].id = 1;
    mpu::mmu[14].id = 0;
    mpu::mmu[15].id = 1;

    mpu::ss_halt_rmask = 0x100;

    memset ( mpu::wram, 0, sizeof (mpu::wram));
    memset ( cart_.sram, 0, sizeof (cart_.sram));
    return 0;
  }

  void call_push_rt ( uint16_t to, uint16_t cur, uint16_t ret )
  {
#if 0
    struct invoke_collector *ic = & mpu::invoke_collector;
    struct invoke_chunk *ch = nullptr;

    if ( ic->elements >= ic->maximum )
    {
      IASSERT (ic->elements == ic->maximum);

      if ( ic->maximum == 0 )
      {
        ic->maximum = 32;
      }
      else
      {
        ic->maximum <<= 1;
      }  
      ch = (struct invoke_chunk *) realloc ( ic->stack, sizeof (struct invoke_chunk) * ic->maximum );

      if ( ch == nullptr )
      { 
        ch = (struct invoke_chunk *) malloc (sizeof (struct invoke_chunk) * ic->maximum);
        IASSERT (ch != nullptr);

        if ( ic->stack != nullptr )
        {
          if ( ic->elements > 0 )
          {
            memcpy ( ch, ic->stack, sizeof (struct invoke_chunk) * ic->elements);
          }
          free (ic->stack);              
        }
      } 
      ic->stack = ch;
    }
    ch = & ic->stack[ic->elements];

    ch->to.address = to;
    ch->to.id = mpu::mmu[to >> 12].id;
    ch->cur.address = cur;
    ch->cur.id = mpu::mmu[cur >> 12].id;
    ch->next.address = ret;
    ch->next.id = mpu::mmu[ret >> 12].id;

    ic->elements++;
#endif 
  }

  IR_FORCEINLINE
  void call_pop_rt ( void )
  {
#if 0
    struct mpu *mpu = & gb::mpu;
    struct invoke_collector *ic = & mpu::invoke_collector;

    if ( ic->elements != 0 )
    { /* PUSH RET_ADDRESS, JP CALL, RET, this kind of calling method will 
       *  make the subroutine call-ret stack unbalanced. This situation exists.
       */
      struct invoke_chunk *ch = & ic->stack[ic->elements - 1];
      struct ir_chunk *ir = & ch->next;
      struct addr_map *am = & mpu::mmu[mpu::pc >> 15];

      if ( ir->address == mpu::pc /* && ir->id == am->id */ )
      {
        ic->elements--;
      }
    }
#endif 
  }

};

template <class T>
struct joypad
{
  uint8_t session[5]; // d0:>
                      // d1:<
                      // d2:^
                      // d3:v 
                      // d4:a
                      // d5:b
                      // d6:select 
                      // d7:start
                      //
                      // [0~3]:gb key state mapper (P1's select mask) [4]:key state (1 for press, opposite of gb...)
  uint8_t *key;

  uint8_t mask; // button sel mask
  uint8_t masks[4];

  uint8_t state;  // d4:select buttons 
                  // d5:select d-pad 
                  // d6:P1's d6 
                  // d7:P1's d7 

  joypad (void)
  {
    session[0] = 0x00;
    session[1] = 0x0f;
    session[2] = 0x0f;
    session[3] = 0x0f;
    session[4] = 0x0f;

    masks[0] = 0xff;
    masks[1] = 0xf0;
    masks[2] = 0x0f;
    masks[3] = 0x00;
  }

  IR_FORCEINLINE
  void reset (void)
  { 
    T *gb = static_cast<T *> (this);

    if ( gb->gb::get_type () == 0 )
    { 
      write ( 0xcf );
    }
    else
    {
      write ( 0xc7 );
    }
  }

  IR_FORCEINLINE
  void write (uint8_t value )
  { 
    uint8_t sel = (value >> 4) & 3;

    mask = masks[sel];
    key = & session[sel];
    state = value & 0xf0;
  }

  IR_FORCEINLINE
  uint8_t read (void) const 
  { 
    return key[0] | state;
  }

  IR_FORCEINLINE
  void update (void)
  { 
    T *gb = static_cast<T *> (this);

    session[4] = 0;

    gb->os_joyp_update ( & session[4] );

    session[1] = session[4] >> 4;
    session[1]^= 0x0f;
    session[2] = session[4] & 0xf;
    session[2]^= 0x0f;
  }
};

template <class T>
struct cart 
{
  enum MBC 
  {
    MBC_NORMAL = 0
  , MBC_MBC1
  , MBC_MBC2
  , MBC_MBC3
  , MBC_MBC4
  , MBC_MBC5
  , MBC_MBC6
  , MBC_MBC7
  , MBC_MMM01
  , MBC_M161
  , MBC_HUC1
  , MBC_HUC3
  , MBC_EXT
  };

  IR_FORCEINLINE  uint8_t 
  read (uint16_t address)
  {
    return ((* this).*_Q_read) (address);
  }

  IR_FORCEINLINE uint8_t 
  read2 (uint16_t address)
  {
    return ((* this).*_Q_read2) (address);
  }

  IR_FORCEINLINE void 
  write (uint16_t address, uint8_t value)
  {
    return ((* this).*_Q_write) (address, value);
  }

  IR_FORCEINLINE void 
  reset (void)
  {
    return ((* this).*_Q_reset) ();
  }

  struct mbc1
  {
    uint32_t state; // d0: ram enable mask (combine sram_mask )

    uint32_t pbk_lo5bit; 
    uint32_t pbk_hi2bit; 
    uint32_t sbk_2bit;
    uint32_t pbk;
    uint32_t sbk;
  };

  struct mbc3
  {
  };

  struct mbc5
  {
    uint32_t state; // d0: ram enable mask (combine sram_mask )

    uint32_t pbk_lo8bit; 
    uint32_t pbk_hi1bit; 
    uint32_t sbk_4bit;
    uint32_t pbk;
    uint32_t sbk;
  };

  uint8_t 
  mbc_read (uint16_t address)
  {
    T *gb = static_cast<T *> (this);

    if (cart::sram_type != 0 )
    {
      gb->mpu::read_directly (address);
    }
    return 0;
  }

  void mbc_write (uint16_t address, uint8_t value)
  {
    T *gb = static_cast<T *> (this);

    gb->mpu::write_directly (address, value);
  }

  void mbc_reset (void)
  {
  }

  uint8_t mbc1_read (uint16_t address)
  {
    T *gb = static_cast<T *> (this);

    switch (address >> 12)
    {
    case 0x0A: // 0xA000-0xAFFF 
    case 0x0B: // 0xB000-0xBFFF 
      // This area is used to address external RAM in the cartridge (if any). 
      // External RAM is often battery buffered, 
      // allowing to store game positions or high score tables, 
      // even if the gameboy is turned off, or if the cartridge is removed from the gameboy. 
      // Available RAM sizes are: 2KByte (at A000-A7FF), 8KByte (at A000-BFFF), and 32KByte 
      // (in form of four 8K banks at A000-BFFF).
      if ( (mbc1_.state & 1) == 0)
      {
        return 0xff;
      }
      break;

    case 0x01: // 0x1000-0x1FFF
    case 0x02: // 0x2000-0x2FFF
    case 0x03: // 0x3000-0x3FFF
    case 0x04: // 0x4000-0x4FFF
    case 0x05: // 0x5000-0x5FFF
    case 0x06: // 0x6000-0x6FFF
    case 0x07: // 0x7000-0x7FFF
    case 0x00: // 0x0000-0x0FFF
    case 0x08: // 0x8000-0x8FFF
    case 0x09: // 0x9000-0x9FFF
    case 0x0C: // 0xC000-0xCFFF
    case 0x0D: // 0xD000-0xDFFF
    case 0x0E: // 0xE000-0xEFFF
    case 0x0F: // 0xF000-0xFFFF
      break;

    default:
      IASSERT (0);
      break;
    }
    return gb->mpu::read_directly (address);
  }

  void mbc1_write (uint16_t address, uint8_t value)
  {
    T *gb = static_cast<T *> (this);

    switch (address >> 12)
    {
    case 0x0A: // 0xA000-0xAFFF 
    case 0x0B: // 0xB000-0xBFFF 
      // This area is used to address external RAM in the cartridge (if any). 
      // External RAM is often battery buffered, 
      // allowing to store game positions or high score tables, 
      // even if the gameboy is turned off, or if the cartridge is removed from the gameboy. 
      // Available RAM sizes are: 2KByte (at A000-A7FF), 8KByte (at A000-BFFF), and 32KByte 
      // (in form of four 8K banks at A000-BFFF).
      if ( (mbc1_.state & 1) != 0)
      {
        gb->mpu::write_directly (address, value);
      }
      break;

    case 0x01: // 0x1000-0x1FFF 
      // Before external RAM can be read or written, 
      // it must be enabled by writing to this address space. 
      // It is recommended to disable external RAM after accessing it, 
      // in order to protect its contents from damage during power down of the gameboy. 
      // Usually the following values are used:
      //
      // 00h  Disable RAM (default)
      // 0Ah  Enable RAM
      mbc1_.state &= ~1;

      if ( (value & 0x0f) == 0x0a)
      {  
        mbc1_.state |= (1 & cart::sram_mask);
      }
      break;

    case 0x02: // 0x2000-0x2FFF 
    case 0x03: // 0x3000-0x3FFF 
      // Writing to this address space selects the lower 5 bits of the 
      // ROM Bank Number (in range 01-1Fh). When 00h is written, 
      // the MBC translates that to bank 01h also. That doesn't harm so far, 
      // because ROM Bank 00h can be always directly accessed by reading from 0000-3FFF.
      // But (when using the register below to specify the upper ROM Bank bits), 
      // the same happens for Bank 20h, 40h, and 60h. Any attempt to address these ROM Banks 
      // will select Bank 21h, 41h, and 61h instead.
       
      mbc1_.pbk_lo5bit = value & 0x1f;
      mbc1_.pbk =  mbc1_.pbk_lo5bit | mbc1_.pbk_hi2bit << 5;

      if ( mbc1_.pbk == 0x00
       || mbc1_.pbk == 0x20 
        || mbc1_.pbk == 0x40 
       || mbc1_.pbk == 0x60 )
      {
        mbc1_.pbk++;
      }
      gb->mpu::mmu[4].id = 
      gb->mpu::mmu[5].id = 
      gb->mpu::mmu[6].id = 
      gb->mpu::mmu[7].id = mbc1_.pbk;

      gb->mpu::mmu[4].ptr = & cart::prom[mbc1_.pbk * MEM_16K];
      gb->mpu::mmu[5].ptr = gb->mpu::mmu[4].ptr + MEM_4K;
      gb->mpu::mmu[6].ptr = gb->mpu::mmu[5].ptr + MEM_4K;
      gb->mpu::mmu[7].ptr = gb->mpu::mmu[6].ptr + MEM_4K;
      break;

    case 0x04: // 0x4000-0x4FFF 
    case 0x05: // 0x5000-0x5FFF 

      if ( (mbc1_.state & 8) != 0 )
      { // ram bank mode 
        mbc1_.pbk_hi2bit = 0;
        mbc1_.sbk_2bit = value & 3;
        mbc1_.sbk = mbc1_.sbk_2bit;
      }
      else
      { // rom bank mode 
        mbc1_.pbk_hi2bit = value & 3;
        mbc1_.sbk_2bit = 0;
        mbc1_.sbk = 0;
      }
      mbc1_.pbk =  mbc1_.pbk_lo5bit | mbc1_.pbk_hi2bit << 5;

      if ( mbc1_.pbk == 0x00
       || mbc1_.pbk == 0x20 
        || mbc1_.pbk == 0x40 
       || mbc1_.pbk == 0x60 )
      {
        mbc1_.pbk++;
      }
      gb->mpu::mmu[4].id = 
      gb->mpu::mmu[5].id = 
      gb->mpu::mmu[6].id = 
      gb->mpu::mmu[7].id = mbc1_.pbk;

      gb->mpu::mmu[4].ptr = & cart::prom[mbc1_.pbk * MEM_16K];
      gb->mpu::mmu[5].ptr = gb->mpu::mmu[4].ptr + MEM_4K;
      gb->mpu::mmu[6].ptr = gb->mpu::mmu[5].ptr + MEM_4K;
      gb->mpu::mmu[7].ptr = gb->mpu::mmu[6].ptr + MEM_4K;

      gb->mpu::mmu[10].id = 
      gb->mpu::mmu[11].id = mbc1_.sbk;

      gb->mpu::mmu[10].ptr = & cart::prom[mbc1_.sbk * MEM_8K];
      gb->mpu::mmu[11].ptr = gb->mpu::mmu[10].ptr + MEM_4K;
      break;

    case 0x06: // 0x6000-0x6FFF 
    case 0x07: // 0x7000-0x7FFF 
      mbc1_.state &= ~8;

      if ( (value & 1) != 0 )
      {
        mbc1_.state |= 8;
      }
      break;

    case 0x00: // 0x0000-0x0FFF 
    case 0x08: // 0x8000-0x8FFF 
    case 0x09: // 0x9000-0x9FFF 
    case 0x0C: // 0xC000-0xCFFF 
    case 0x0D: // 0xD000-0xDFFF 
    case 0x0E: // 0xE000-0xEFFF 
    case 0x0F: // 0xF000-0xFFFF 
      break;

    default:
      IASSERT (0);
      break;
    }
  }

  void mbc1_reset (void)
  {
    mbc1_.state = 0;
    mbc1_.pbk_hi2bit = 0;
    mbc1_.pbk_lo5bit = 1;
    mbc1_.pbk = 1;
    mbc1_.sbk = 0;
    mbc1_.sbk_2bit = 0;
  }

  uint8_t ram_disable_bus (void)
  {
    return 0;
  }

  uint8_t mbc5_read (uint16_t address)
  {
    T *gb = static_cast<T *> (this);

    switch (address >> 12)
    {
    case 0x0A: // 0xA000-0xAFFF 
    case 0x0B: // 0xB000-0xBFFF 
      if ( (mbc5_.state & 1) == 0)
      {
        return ram_disable_bus ();
      }
      break;

    case 0x01: // 0x1000-0x1FFF
    case 0x02: // 0x2000-0x2FFF
    case 0x03: // 0x3000-0x3FFF
    case 0x04: // 0x4000-0x4FFF
    case 0x05: // 0x5000-0x5FFF
    case 0x06: // 0x6000-0x6FFF
    case 0x07: // 0x7000-0x7FFF
    case 0x00: // 0x0000-0x0FFF
    case 0x08: // 0x8000-0x8FFF
    case 0x09: // 0x9000-0x9FFF
    case 0x0C: // 0xC000-0xCFFF
    case 0x0D: // 0xD000-0xDFFF
    case 0x0E: // 0xE000-0xEFFF
    case 0x0F: // 0xF000-0xFFFF
      break;

    default:
      IASSERT (0);
      break;
    }
    return gb->mpu::read_directly (address);
  }

  void mbc5_write (uint16_t address, uint8_t value)
  {
    T *gb = static_cast<T *> (this);

    switch (address >> 12)
    {
    case 0x0A: // 0xA000-0xAFFF 
    case 0x0B: // 0xB000-0xBFFF 
      // This area is used to address external RAM in the cartridge (if any). 
      // External RAM is often battery buffered, 
      // allowing to store game positions or high score tables, 
      // even if the gameboy is turned off, or if the cartridge is removed from the gameboy. 
      // Available RAM sizes are: 2KByte (at A000-A7FF), 8KByte (at A000-BFFF), and 32KByte 
      // (in form of four 8K banks at A000-BFFF).
      if ( (mbc5_.state & 1) != 0)
      {
        gb->mpu::write_directly (address, value);
      }
      break;

    case 0x00: // 0x0000-0x0fff
    case 0x01: // 0x1000-0x1FFF 
      mbc5_.state &= ~1;

      if ( (value & 0x0f) == 0x0a)
      {  
        mbc5_.state |= (1 & cart::sram_mask);
      }
      break;

    case 0x02: // 0x2000-0x2FFF 
      mbc5_.pbk_lo8bit = value;
      mbc5_.pbk = mbc5_.pbk_hi1bit & 1;
      mbc5_.pbk <<= 8;
      mbc5_.pbk |= mbc5_.pbk_lo8bit;

      gb->mpu::mmu[4].id = 
      gb->mpu::mmu[5].id = 
      gb->mpu::mmu[6].id = 
      gb->mpu::mmu[7].id = mbc5_.pbk;

      gb->mpu::mmu[4].ptr = & cart::prom[mbc5_.pbk * MEM_16K];
      gb->mpu::mmu[5].ptr = gb->mpu::mmu[4].ptr + MEM_4K;
      gb->mpu::mmu[6].ptr = gb->mpu::mmu[5].ptr + MEM_4K;
      gb->mpu::mmu[7].ptr = gb->mpu::mmu[6].ptr + MEM_4K;
      break;

    case 0x03: // 0x3000-0x3FFF 
      mbc5_.pbk_hi1bit = value;
      mbc5_.pbk = mbc5_.pbk_hi1bit & 1;
      mbc5_.pbk <<= 8;
      mbc5_.pbk |= mbc5_.pbk_lo8bit;

      gb->mpu::mmu[4].id = 
      gb->mpu::mmu[5].id = 
      gb->mpu::mmu[6].id = 
      gb->mpu::mmu[7].id = mbc5_.pbk;

      gb->mpu::mmu[4].ptr = & cart::prom[mbc5_.pbk * MEM_16K];
      gb->mpu::mmu[5].ptr = gb->mpu::mmu[4].ptr + MEM_4K;
      gb->mpu::mmu[6].ptr = gb->mpu::mmu[5].ptr + MEM_4K;
      gb->mpu::mmu[7].ptr = gb->mpu::mmu[6].ptr + MEM_4K;
      break;

    case 0x04: // 0x4000-0x4FFF 
    case 0x05: // 0x5000-0x5FFF 

      mbc5_.sbk_4bit = value;
      mbc5_.sbk = mbc5_.sbk_4bit & 0x0f;

      gb->mpu::mmu[10].id = 
      gb->mpu::mmu[11].id = mbc5_.sbk;

      gb->mpu::mmu[10].ptr = & cart::sram[mbc5_.sbk * MEM_8K];
      gb->mpu::mmu[11].ptr = gb->mpu::mmu[10].ptr + MEM_4K;
      break;

    case 0x06: // 0x6000-0x6FFF 
    case 0x07: // 0x7000-0x7FFF 
    case 0x08: // 0x8000-0x8FFF 
    case 0x09: // 0x9000-0x9FFF 
    case 0x0C: // 0xC000-0xCFFF 
    case 0x0D: // 0xD000-0xDFFF 
    case 0x0E: // 0xE000-0xEFFF 
    case 0x0F: // 0xF000-0xFFFF 
      break;

    default:
      IASSERT (0);
      break;
    }
  }

  void mbc5_reset (void)
  {
    mbc5_.state = 0;
    mbc5_.pbk_hi1bit = 0;
    mbc5_.pbk_lo8bit = 1;
    mbc5_.pbk = 1;
    mbc5_.sbk = 0;
    mbc5_.sbk_4bit = 0;
  }

  int 
  load ( uint8_t *memory, uint32_t bytes, uint32_t force_dmg_mask = 0 )
  {
    T *gb = static_cast<T *> (this);

    uint32_t prom_bytes = 0;
    uint32_t sram_bytes = 0;
    uint32_t prom_banks = 0;
    uint32_t sram_banks = 0;
    uint32_t sram_type = 0;
    uint32_t rtc_mask = 0;

    IASSERT (memory != nullptr);
    IASSERT (bytes != 0);

    MBC mbc = MBC_NORMAL;

    static const uint8_t logo[] =
    {
      0xce, 0xed, 0x66, 0x66, 0xcc, 0x0d, 0x00, 0x0b
    , 0x03, 0x73, 0x00, 0x83, 0x00, 0x0c, 0x00, 0x0d
    , 0x00, 0x08, 0x11, 0x1f, 0x88, 0x89, 0x00, 0x0e
    , 0xdc, 0xcc, 0x6e, 0xe6, 0xdd, 0xdd, 0xd9, 0x99
    , 0xbb, 0xbb, 0x67, 0x63, 0x6e, 0x0e, 0xec, 0xcc
    , 0xdd, 0xdc, 0x99, 0x9f, 0xbb, 0xb9, 0x33, 0x3e
    };

    if ( bytes <= 0x14f )
    {
      return 1;
    }
    else if ( bytes > MEM_1MB * 8 )
    {
      return 2;
    }
    else if ( ::memcmp ( & memory[0x104], & logo[0], sizeof (logo) ) != 0 )
    {
      return 3;
    }

    switch ( memory[0x147] )
    {
    case 0x00:
      mbc = MBC_NORMAL;

      _Q_read = & cart<T>::mbc_read;
      _Q_read2= & cart<T>::mbc_read;
      _Q_write= & cart<T>::mbc_write;
      _Q_reset= & cart<T>::mbc_reset;

      sram_type = 0;
      break;

    case 0x01:
      mbc = MBC_MBC1;

      _Q_read = & cart<T>::mbc1_read;
      _Q_read2= & cart<T>::mbc1_read;
      _Q_write= & cart<T>::mbc1_write;
      _Q_reset= & cart<T>::mbc1_reset;

      sram_type = 0;
      break;

    case 0x02:
      mbc = MBC_MBC1;

      _Q_read = & cart<T>::mbc1_read;
      _Q_read2= & cart<T>::mbc1_read;
      _Q_write= & cart<T>::mbc1_write;
      _Q_reset= & cart<T>::mbc1_reset;

      sram_type = 1;
      break;

    case 0x03:
      mbc = MBC_MBC1;

      _Q_read = & cart<T>::mbc1_read;
      _Q_read2= & cart<T>::mbc1_read;
      _Q_write= & cart<T>::mbc1_write;
      _Q_reset= & cart<T>::mbc1_reset;

      sram_type = 2;
      break;

    case 0x05:
      mbc = MBC_MBC2;

      sram_type = 0;
      break;

    case 0x06:
      mbc = MBC_MBC2;

      sram_type = 2;
      break;

    case 0x08:
    case 0x09:
      return 4;

    case 0x0B:
      mbc = MBC_MMM01;

      sram_type = 0;
      break;

    case 0x0C:
      mbc = MBC_MMM01;

      sram_type = 1;
      break;

    case 0x0D:
      mbc = MBC_MMM01;

      sram_type = 2;
      break;


     //  $0F	MBC3+TIMER+BATTERY
     //  $10	MBC3+TIMER+RAM+BATTERY 10
     //  $11	MBC3
     //  $12	MBC3+RAM 10
     //  $13	MBC3+RAM+BATTERY 10


    case 0x0f: // MBC3+TIMER+BATTERY
    case 0x10: // MBC3+TIMER+RAM+BATTERY
      rtc_mask = UINT32_MAX;
      break;

    case 0x1b: // mbc5+ram+battery

      mbc = MBC_MBC5;

      _Q_read = & cart<T>::mbc5_read;
      _Q_read2= & cart<T>::mbc5_read;
      _Q_write= & cart<T>::mbc5_write;
      _Q_reset= & cart<T>::mbc5_reset;

      sram_type = 2;
      break;

    default:
      return 5;
    }

    switch ( memory[0x148] )
    {
    case 0x00:
      prom_banks = 2;
      break;

    case 0x01:
      prom_banks = 4;
      break;

    case 0x02:
      prom_banks = 8;
      break;

    case 0x03:
      prom_banks = 16;
      break;

    case 0x04:
      prom_banks = 32;
      break;

    case 0x05:
      prom_banks = 64;
      break;

    case 0x06:
      prom_banks = 128;
      break;

    case 0x07:
      prom_banks = 256;
      break;

    case 0x08:
      prom_banks = 512;
      break;

    case 0x52:
      prom_banks = 72;
      break;

    case 0x53:
      prom_banks = 80;
      break;

    case 0x54:
      prom_banks = 96;
      break;

    default:
      return 6;
    }

    switch ( memory[0x148] )
    {
    case 0x00:
      sram_banks = 0;
      break;

    case 0x01:
    case 0x02:
      sram_banks = 1;
      break;

    case 0x03:
      sram_banks = 4;
      break;

    case 0x04:
      sram_banks = 16;
      break;

    case 0x05:
      sram_banks = 8;
      break;

    default:
      return 7;
    }
    cart::prom_bank = prom_banks;
    cart::sram_bank = sram_banks;
    cart::prom_byte = prom_banks * MEM_16K;
    cart::sram_byte = sram_banks * MEM_8K;
    cart::sram_type = sram_type;
    cart::sram_mask = (sram_type != 0) ? UINT32_MAX : 0;
    cart::mbc = mbc;
    

    fill_gbyte ( & cart::prom[0], memory, bytes, prom_banks * MEM_16K );
    fill_gbyte ( & cart::sram[0], nullptr, 0, sram_banks * MEM_8K );

    gb->gb::state = 0x80000000; // set suspend
    gb->gb::state|= 0x4000;

    if ( memory[0x143] == 0xc0 
      || ( memory[0x143] == 0x80 
       && force_dmg_mask == 0 ) )
    {
      gb->gb::state |= 1;
    }
    return 0;
  }

  IR_STATIC_FORCEINLINE
  void fill_gbyte ( void *nbyte, uint8_t *src, uint32_t copy_bytes, uint32_t fill_bytes)
  {
  #ifdef IR_FAST_X86_SIMD_VECTOR
    uint32_t block = copy_bytes >> 6;
    uint32_t sblock= copy_bytes & 63;
    uint32_t block2= (fill_bytes - copy_bytes) >> 6;
    uint8_t *dst = reinterpret_cast<uint8_t *> (nbyte);

    while (block != 0)
    {
      __m256i v = _mm256_loadu_si256 (reinterpret_cast<const __m256i *> ( src) + 0);
      __m256i v2= _mm256_loadu_si256 (reinterpret_cast<const __m256i *> ( src) + 1);
      __m128i w = _mm256_castsi256_si128 (v);
      __m128i w2= _mm256_extractf128_si256 (v, 1);
      __m128i w3= _mm256_castsi256_si128 (v2);
      __m128i w4= _mm256_extractf128_si256 (v2, 1);
      __m256i q = _mm256_cvtepu8_epi16 (w);
      __m256i q2= _mm256_cvtepu8_epi16 (w2);
      __m256i q3= _mm256_cvtepu8_epi16 (w3);
      __m256i q4= _mm256_cvtepu8_epi16 (w4);

      _mm256_storeu_si256 (reinterpret_cast<__m256i *> ( dst) + 0, q);
      _mm256_storeu_si256 (reinterpret_cast<__m256i *> ( dst) + 1, q2);
      _mm256_storeu_si256 (reinterpret_cast<__m256i *> ( dst) + 2, q3);
      _mm256_storeu_si256 (reinterpret_cast<__m256i *> ( dst) + 3, q4);

      dst += 128;
      src += 64;

      block--;
    }

    while (sblock != 0)
    {
      dst[0] = src[0];
      dst[1] = 0;

      dst++;
      dst++;

      src++;

      sblock--;
    }

    if (block2 != 0)
    {
      __m256i z = _mm256_setzero_si256 ();

      do
      {
        _mm256_storeu_si256 (reinterpret_cast<__m256i *> ( dst) + 0, z);
        _mm256_storeu_si256 (reinterpret_cast<__m256i *> ( dst) + 1, z);
        _mm256_storeu_si256 (reinterpret_cast<__m256i *> ( dst) + 2, z);
        _mm256_storeu_si256 (reinterpret_cast<__m256i *> ( dst) + 3, z);

        dst += 128;
      } while (--block2 != 0);
    }

  #else 
    uint32_t block = copy_bytes >> 6;
    uint32_t block2= (fill_bytes - copy_bytes) >> 6;
    uint8_t *dst = reinterpret_cast<uint8_t *> (nbyte);

    while (block != 0)
    {
      dst[0] = src[0];
      dst[1] = 0;

      dst++;
      dst++;

      src++;

      block--;
    }

    while (block2 != 0)
    {
      dst[0] = 0;
      dst[1] = 0;

      dst++;
      dst++;

      block2--;
    }
  #endif 
  }

public:
  struct nbyte prom[MEM_1MB * 8];
  struct nbyte sram[MEM_32K * 8];

  uint32_t prom_byte; 
  uint32_t sram_byte; 
  uint32_t prom_bank; 
  uint32_t sram_bank; 
  uint32_t sram_type; // 0:none 1:ram 2:battery 
  uint32_t sram_mask; // sram_type != 0 ? 0xffffffff : 0 
  uint32_t rtc_mask; // 



  MBC mbc;

  union
  {
    mbc1 mbc1_;
    mbc5 mbc5_;
  };

  uint8_t (cart<T>::*_Q_read)(uint16_t address);
  uint8_t (cart<T>::*_Q_read2)(uint16_t address);
  void (cart<T>::*_Q_write)(uint16_t address, uint8_t value);
  void (cart<T>::*_Q_reset)(void);
};

template <class T>
struct timer 
{ // FF04 - DIV: Divider register
  // This register is incremented at a rate of 16384Hz (~16779Hz on SGB). 
  // Writing any value to this register resets it to $00. Additionally, 
  // this register is reset when executing the stop instruction, 
  // and only begins ticking again once stop mode ends. This also occurs during a speed switch. 
  // (TODO: how is it affected by the wait after a speed switch?)
  // 
  // Note: The divider is affected by CGB double speed mode, and will increment at 32768Hz in double speed.
  // 
  // FF05 - TIMA: Timer counter
  // This timer is incremented at the clock frequency specified by the TAC register ($FF07). 
  // When the value overflows (exceeds $FF) it is reset to the value specified in TMA (FF06) and an interrupt is requested, as described below.
  // 
  // FF06 - TMA: Timer modulo
  // When TIMA overflows, it is reset to the value in this register and an interrupt is requested. 
  // Example of use: if TMA is set to $FF, an interrupt is requested at the clock frequency selected
  // in TAC (because every increment is an overflow). However, if TMA is set to $FE, an interrupt is only 
  // requested every two increments, which effectively divides the selected clock by two. 
  // Setting TMA to $FD would divide the clock by three, and so on.
  // 
  // If a TMA write is executed on the same M-cycle as the content of TMA is transferred to TIMA due to a timer overflow, 
  // the old value is transferred to TIMA.
  // 
  // FF07 - TAC: Timer control
  // .--------------------------------------------------------.
  // |       | 7  6  5  4  3  |       2        |     1  0     |
  // |--------------------------------------------------------|
  // | TAC   |                |     Enable     | Clock select |
  // *--------------------------------------------------------*
  // 
  // Enable: Controls whether TIMA is incremented. Note that DIV is always counting, regardless of this bit.
  // 
  // Clock select: Controls the frequency at which TIMA is incremented, as follows:
  // 
  // .------------------------------------------------------------------------------------------------------------.
  // | Clock select | Increment every |                            Frequency (Hz)                                 |
  // |--------------|---------------------------------------------------------------------------------------------|
  // |              |                 | DMG, SGB2, CGB in single-speed mode |   SGB1   | CGB in double-speed mode |
  // |------------------------------------------------------------------------------------------------------------|
  // |       00     |   256 M-cycles  |             4096                    | ~4194    |     8192                 |
  // |       01     |   4 M-cycles    |             262144                  |  ~268400 |     524288               |
  // |       10     |   16 M-cycles   |             65536                   | ~67110   |     131072               |
  // |       11     |   64 M-cycles   |             16384                   | ~16780   |     32768                |
  // *------------------------------------------------------------------------------------------------------------*  
  uint32_t timer_clk;
  uint32_t timer_blk_;
  uint32_t timer_blk; // mask   0xff, mixed enable mask
  uint32_t timer_tma_x;
  uint32_t timer_tma_y;
  uint32_t timer_sft;
  uint32_t timer_tma;
  uint32_t timer_tac;
  uint32_t timer_lut[4]; // clk, shift

  struct 
  {
  #if ENDIAN9 == 0
    struct 
    {
      uint8_t unused;
      uint8_t div;
      uint8_t unused2;
      uint8_t unused3;
    };
    uint32_t div_clk;
  #elif ENDIAN9 == 1
    struct
    {
      uint8_t unused;  
      uint8_t unused2;
      uint8_t div;
      uint8_t unused3;
    };
    uint32_t div_clk;
  #endif 
  };

  timer (void)
  {
    timer_lut[0] = 10;
    timer_lut[1] = 4;
    timer_lut[2] = 6;
    timer_lut[3] = 8;
  }

  IR_FORCEINLINE  void 
  clk (int32_t clk)
  {
    T *gb = static_cast<T *> (this);
    
    div_clk += clk;
    timer_clk += clk;

    if ( (timer_clk & timer_blk) != 0 )
    { // timer overflow
      timer_clk -= timer_blk; 
      timer_clk %= timer_tma_y;
      timer_clk += timer_tma_x;

      gb->gb::interrupt_requset (INT_TIMER, UINT32_MAX);
    }
  }

  uint8_t 
  divider_read (void) const 
  { 
    return div;
  }

  void divider_write (uint8_t value) 
  { // writing any value to this register resets it to $00
    div_clk = 0;

    timer_clk &= ~((timer_blk_ >> 8) - 1); // clear tiny, miscellaneous tick 
  }

  IR_FORCEINLINE
  void tima_write ( uint8_t value )
  { 
    timer_clk = value;
    timer_clk<<= timer_sft;
  }

  IR_FORCEINLINE
  void tma_write ( uint8_t value )
  { 
    timer_tma_x = value;
    timer_tma_x<<= timer_sft;
    timer_tma_y = timer_blk_ - timer_tma_x;
    timer_tma = value;
  }

  IR_FORCEINLINE
  void tac_write ( uint8_t value )
  { 
    uint8_t m;

    if ( ( (m = (timer_tac ^ value)) & 7) != 0 )
    {
      if ( (m & 3) != 0 )
      {
        timer_sft = timer_lut[value & 3];

        timer_blk = 256 << timer_sft;
        timer_blk_= timer_blk;

        timer_tma_x = timer_tma << timer_sft;
        timer_tma_y = timer_blk_ - timer_tma_x;

        timer_clk&= (1 << timer_sft) - 1;
        timer_clk = timer_tma_x;
      }

      if ( (m & 4) != 0)
      { 
        if ( (value & 4) != 0 )
        { // timer open 
          timer_clk = timer_tma_x;
          timer_blk = timer_blk_;
        }
        else
        { // timer close 
          timer_blk = 0;
        }      
      }
    }
    timer_tac = value;
  }

  IR_FORCEINLINE
  uint8_t tima_read (void) const 
  { 
    return (timer_clk >> timer_sft) & 0xff;
  }

  IR_FORCEINLINE
  uint8_t tma_read (void) const 
  { 
    return timer_tma;
  }

  IR_FORCEINLINE
  uint8_t tac_read (void) const 
  { 
    return timer_tac;
  }

  void reset (void)
  {
    timer_clk = 0;

    timer_sft = timer_lut[0];

    timer_blk = 0;
    timer_blk_= 256 << timer_sft;

    timer_tma_x = 0;
    timer_tma_x<<= timer_sft;
    timer_tma_y = timer_blk_ - timer_tma_x;
    timer_tma = 0;

    timer_tac = 0;

    div_clk = 0;
  }
};

template <class T>
struct apu
{
  apu (void) : sample_reload (0)
  {
    static const uint8_t _s_duty[] =
    { 
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff
    , 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff
    , 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff    
    , 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00
    , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    for ( uintptr_t id = 0; id != sizeof (_s_duty); id++ )
    {
      apu::duty[id] = _s_duty[id];
    }
    divisor[0] = 8;    // noise channel divisor 
    divisor[1] = 16;   // noise channel divisor 
    divisor[2] = 32;   // noise channel divisor 
    divisor[3] = 48;   // noise channel divisor 
    divisor[4] = 64;   // noise channel divisor 
    divisor[5] = 80;   // noise channel divisor 
    divisor[6] = 96;   // noise channel divisor 
    divisor[7] = 112;  // noise channel divisor 

    noi_chan_.xor_tab[0] = 0;      // 00 -lfsr 15bit 
    noi_chan_.xor_tab[1] = 0x8000; // 01 -lfsr 15bit 
    noi_chan_.xor_tab[2] = 0x8000; // 10 -lfsr 15bit 
    noi_chan_.xor_tab[3] = 0;      // 11 -lfsr 15bit 
    noi_chan_.xor_tab[4] = 0;      // 00 -lfsr 7bit 
    noi_chan_.xor_tab[5] = 0x80;   // 01 -lfsr 7bit 
    noi_chan_.xor_tab[6] = 0x80;   // 10 -lfsr 7bit 
    noi_chan_.xor_tab[7] = 0;      // 11 -lfsr 7bit 

    squ_chans[0].duty = & apu::duty[0];
    squ_chans[1].duty = & apu::duty[0];

    squ_chans[0].regs[0].or_mask = 0x80;
    squ_chans[0].regs[1].or_mask = 0x3f;
    squ_chans[0].regs[2].or_mask = 0x00;
    squ_chans[0].regs[3].or_mask = 0xff;
    squ_chans[0].regs[4].or_mask = 0xbf;

    squ_chans[1].regs[0].or_mask = 0xff;
    squ_chans[1].regs[1].or_mask = 0x3f;
    squ_chans[1].regs[2].or_mask = 0x00;
    squ_chans[1].regs[3].or_mask = 0xff;
    squ_chans[1].regs[4].or_mask = 0xbf;

    wav_chan_.regs[0].or_mask = 0x7f;
    wav_chan_.regs[1].or_mask = 0xff;
    wav_chan_.regs[2].or_mask = 0x9f;
    wav_chan_.regs[3].or_mask = 0xff;
    wav_chan_.regs[4].or_mask = 0xbf;

    noi_chan_.regs[0].or_mask = 0xff;
    noi_chan_.regs[1].or_mask = 0xff;
    noi_chan_.regs[2].or_mask = 0x00;
    noi_chan_.regs[3].or_mask = 0x00;
    noi_chan_.regs[4].or_mask = 0xbf;
  }

  struct len_ctr
  {
    uint32_t timer;
    uint32_t enable_mask;

    IR_FORCEINLINE
    void reset (void)
    {
      timer = 8;
      enable_mask = 0;
    }
  };

  struct sweep
  { 
    uint32_t timer; // timer 
    uint32_t load;   // overload 
    uint32_t load_8x;// 8 period version.

    uint32_t freq; // shadow freq 

    uint32_t post; // direction (post ? UINT32_MAX : 0)

    uint32_t neg_work; // have done at least one negative sweep mask 
    uint32_t shift; // 2^shift

    uint32_t enable_mask; // enable mask (UINT32_MAX or 0)

    IR_FORCEINLINE
    void reset (void)
    {
      timer = 

      load =
      load_8x = 8;

      enable_mask = 0;
    }

    IR_STATIC_FORCEINLINE uint32_t 
    scanning (sweep &sweep)
    {
      uint32_t freq = sweep.freq;
      uint32_t delta = freq >> sweep.shift;

      if ( sweep.post != 0 )
      {
        freq += delta;
      }
      else
      {
        freq -= delta;

        sweep.neg_work = UINT32_MAX;
      }
      return freq;
    }
  };

  struct envelope
  { 
    uint8_t vol; // volume
    uint8_t vec; // 1 or 0xff 
    uint8_t timer; 
    uint8_t load; // n * (1/64) seconds 
    uint8_t load_8x;// 8 period version.

    uint32_t enable_mask;

    IR_FORCEINLINE
    void reset (void)
    {
      vol = 0;
      vec = 1;

      timer = 

      load = 
      load_8x = 8;

      enable_mask = 0;
    }
  };

  struct reg
  {
    uint8_t io;
    uint8_t or_mask;
  };

  struct chan : public len_ctr
  {
    int32_t timer;
    int32_t load;

    uint32_t enable_mask; // enable ? UINT32_MAX : 0 
    uint32_t dac_mask; // enable ? UINT32_MAX : 0 

    uint16_t s0_mask; // enable ? UINT32_MAX : 0
    uint16_t s1_mask; // enable ? UINT32_MAX : 0

    reg regs[5];

    uint8_t *duty;       // squ, squ2
    uint32_t sft;        //            wav
    uint32_t pos;        // squ, squ2, wav 


    IR_FORCEINLINE
    void reset (void)
    {
      len_ctr::reset ();

      timer = 
      load = 12000;

      enable_mask = 
      dac_mask = 0;

      s0_mask = 
      s1_mask = 0;

      sft = 8;
    }
  };

  struct squ_chan : public chan
                  , public envelope 
                  , public sweep
  {
    IR_FORCEINLINE void 
    tick (int32_t tick)
    {
      if ( (chan::timer = chan::timer - tick) <= 0 )
      { // update output
        chan::pos++;
        chan::pos -= chan::timer / chan::load;

        chan::timer %= chan::load;
        chan::timer += chan::load;
      }
    }

    IR_FORCEINLINE uint8_t  
    get_output (void) const 
    {
      return chan::duty[chan::pos & 7] & envelope::vol;
    }
    
    IR_FORCEINLINE void  
    reset (void) 
    {
      chan::reset ();
    }
  };

  struct wav_chan : public chan
  {
    uint8_t table[32];

    IR_FORCEINLINE uint8_t  
    get_output (void) const 
    {
      return wav_chan::table[chan::pos & 31] >> chan::sft;
    }

    IR_FORCEINLINE void 
    tick (int32_t tick)
    {
      if ( (chan::timer = chan::timer - tick) <= 0 )
      { // update output 
        chan::pos++;
        chan::pos -= chan::timer / chan::load;

        chan::timer %= chan::load;
        chan::timer += chan::load;
      }
    }

    IR_FORCEINLINE void  
    reset (void) 
    {
      chan::reset ();
    }
  };

  struct noi_chan : public chan
                  , public envelope  
  {
    uint16_t lfsr;        
    uint16_t xor_tab[8];  
    uint16_t *xor_2bit;  

    IR_FORCEINLINE uint8_t  
    get_output (void) const 
    {
      return (lfsr & 1) - 1  & chan::enable_mask & envelope::vol;
    }

    IR_FORCEINLINE void 
    tick (int32_t tick)
    {
      if ( (chan::timer = chan::timer - tick) <= 0 )
      { // update output 
        uint32_t iterate  = 1;
                 iterate -= chan::timer / chan::load;
                    
        chan::timer %= chan::load; 
        chan::timer += chan::load;

        do
        {
       #if 1
          lfsr  |= xor_2bit[lfsr & 3];
          lfsr >>= 1;     
          // 00A7A1E1 8B 86 A4 01 00 00    mov         eax,dword ptr [esi+1A4h]  ;; eax := chan::xor_2bit
          // 00A7A1E7 8B CA                mov         ecx,edx                   ;; ecx := chan::lfsr
          // 00A7A1E9 83 E1 03             and         ecx,3                     ;; ecx := chan::lfsr & 3
          // 00A7A1EC 66 8B 04 48          mov         ax,word ptr [eax+ecx*2]   ;;  ax := xor_2bit_res 
          // 00A7A1F0 66 0B C2             or          ax,dx                     ;;  ax := xor_2bit_res | lfsr
          // 00A7A1F3 66 D1 E8             shr         ax,1                      ;;  ax := ax >> 1
          // 00A7A1F6 0F B7 D0             movzx       edx,ax                    ;; edx := write back 
          // 00A7A1F9 66 89 96 90 01 00 00 mov         word ptr [esi+190h],dx    ;;  dx := write back lfsr memory ???????....
          // 00A7A200 4F                   dec         edi  
          // 00A7A201 75 DE                jne         $LN3210+4348h (0A7A1E1h)  ;; release x86 vs2012 avx
       #endif 
        } while (--iterate != 0);
      }
    }

    IR_FORCEINLINE void  
    reset (void) 
    {
      chan::reset ();

      xor_2bit = & xor_tab[0];

      lfsr = 0x7fff;
    }
  };

  IR_FORCEINLINE void 
  nr10_write (uint8_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // sweep frequency 
      // 
      //  Bit 6-4 - Sweep Time
      //  Bit 3   - Sweep Increase/Decrease
      //             0: Addition    (frequency increases)
      //             1: Subtraction (frequency decreases)
      //  Bit 2-0 - Number of sweep shift (n: 0-7)
      // Sweep Time:
      // 
      //  000: sweep off - no freq change
      //  001: 7.8 ms  (1/128Hz)
      //  010: 15.6 ms (2/128Hz)
      //  011: 23.4 ms (3/128Hz)
      //  100: 31.3 ms (4/128Hz)
      //  101: 39.1 ms (5/128Hz)
      //  110: 46.9 ms (6/128Hz)
      //  111: 54.7 ms (7/128Hz)
      // 
      // The change of frequency (NR13,NR14) at each shift is calculated by\
      //  the following formula where X(0) is initial freq & X(t-1) is last freq:
      // 
      //  X(t) = X(t-1) +/- X(t-1)/2^n
      squ_chan &squ = squ_chans[0];

      sweep &sweep = squ_chans[0];
      reg &reg = squ.regs[0];

      sweep.load = (value & 0x70) >> 4;    
      sweep.load_8x = (sweep.load == 0) ? 8 : sweep.load;
      sweep.post = (value & 8) ^ 8;
      sweep.shift = value & 7;

      if ( (sweep.post & sweep.neg_work) != 0 )
      {
        squ.chan::enable_mask = 0;
      }
      reg.io = value;
    }
  }

  IR_FORCEINLINE void 
  nrx1_write (squ_chan &squ, uint8_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // Sound length/Wave pattern duty (R/W)
      //  Bit 7-6 - Wave Pattern Duty (Read/Write)
      //  Bit 5-0 - Sound length data (Write Only) (t1: 0-63)
      // Wave Duty:
      // 
      //  00: 12.5% ( _-------_-------_------- )
      //  01: 25%   ( __------__------__------ )
      //  10: 50%   ( ____----____----____---- ) (normal)
      //  11: 75%   ( ______--______--______-- )
      len_ctr &len_ctr = squ;
      reg &reg = squ.regs[1];

      len_ctr.timer = 64 - (value & 63);

      squ.duty = & apu::duty[(value >> 6) << 3];

      if ( squ.chan::enable_mask == 0 )
      {
        squ.duty = & apu::duty[32];
      }
      reg.io = value;
    }
  }

  IR_FORCEINLINE void 
  nrx2_write (squ_chan &squ, uint8_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // volume/envelope 
      //    Bit 7-4 - Initial Volume of envelope (0-0Fh) (0=No Sound)
      //  Bit 3   - Envelope Direction (0=Decrease, 1=Increase)
      //  Bit 2-0 - Number of envelope sweep (n: 0-7)
      //            (If zero, stop envelope operation.)
      // Length of 1 step = n*(1/64) seconds
      reg &reg = squ.regs[2];

      squ.chan::dac_mask = UINT32_MAX;

      if ( (value & 0xf8) == 0 )
      { // close channel's DAC
        squ.chan::enable_mask = 0;
        squ.chan::dac_mask = 0;

        squ.duty = & apu::duty[32];       
      }
      reg.io = value;
    }
  }

  IR_FORCEINLINE void 
  nrx3_write (squ_chan &squ, uint32_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // Lower 8 bits of 11 bit frequency (x). 
      // Next 3 bit are in NR14 ($FF14) 
      reg &nrx3 = squ.regs[3];
      reg &nrx4 = squ.regs[4];

      uint32_t freq = nrx4.io & 7;
               freq<<= 8;
               freq |= value;

      // channel freq:1048576 / (2048 - freq_11bit) 
      // range:1048576~512 hz (131072~64 hz for tone)
      // master:4194304 hz := 1048576 * 4 ~ 512 * 8192
      // 
      //            1048576                  2048 - f
      // 4194304 / ----------- => 4194304 * ----------- => (2048 - f) * 4
      //            2048 - f                 1048576
      squ.chan::load = (2048 - freq) << 2;

      nrx3.io = value;
    }
  }

  IR_FORCEINLINE void 
  nrx4_write (struct squ_chan &squ, uint32_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // Bit 7   - Initial (1=Restart Sound)     (Write Only)
      // Bit 6   - Counter/consecutive selection (Read/Write)
      //           (1=Stop output when length in NR11 expires)
      // Bit 2-0 - Frequency's higher 3 bits (x) (Write Only) 
      len_ctr &len_ctr = squ;
      sweep &sweep = squ;
      envelope &envelope = squ;

    //reg &nrx0 = squ.regs[0];
      reg &nrx1 = squ.regs[1];
      reg &nrx2 = squ.regs[2];
      reg &nrx3 = squ.regs[3];
      reg &nrx4 = squ.regs[4];

      uint32_t freq = value & 7;
               freq<<= 8;
               freq |= nrx3.io;

      squ.chan::load = (2048 - freq) << 2;

     // printf ("nrx4:write:%02x \n", value );

      if ( static_cast <int8_t> (value) < 0 )
      { // Writing a value to NRx4 with bit 7 set causes the following things to occur:
        // 
        // Channel is enabled.
        // If length counter is zero, it is set to 64 (256 for wave channel).
        // Frequency timer is reloaded with period.
        // Volume envelope timer is reloaded with period.
        // Channel volume is reloaded from NRx2.
        // Square 1's sweep does several things (see frequency sweep).
        // => Square 1's frequency is copied to the shadow register.
        // => The sweep timer is reloaded.
        // => The internal enabled flag is set if either the sweep period or shift are non-zero, cleared otherwise.
        // => If the sweep shift is non-zero, frequency calculation and the overflow check are performed immediately.
        // 
        // Note that if the channel's DAC is off, after the above actions occur the channel will be immediately disabled again. 
        squ.chan::enable_mask = squ.chan::dac_mask;
        squ.chan::timer = squ.chan::load;

        squ.duty = & apu::duty[nrx1.io >> 6 << 3];

        envelope.load = nrx2.io & 7;
        envelope.load_8x = (envelope.load == 0) ? 8 : envelope.load;
        envelope.timer = envelope.load_8x;
        envelope.vol = nrx2.io >> 4;
        envelope.vec = ((nrx2.io & 8) != 0) ? 0x01 : UINT8_MAX;
        envelope.enable_mask = envelope.load != 0 ? UINT32_MAX : 0;

        sweep.enable_mask = 0;
        sweep.freq = freq;

        sweep.load_8x = (sweep.load == 0) ? 8 : sweep.load;
        sweep.timer = sweep.load_8x;
        sweep.neg_work = 0;
 
        len_ctr.enable_mask = 
            (value & 0x40) ? UINT32_MAX : 0;

        if ( squ.chan::enable_mask == 0 )
        { // disable channel 
          squ.duty = & apu::duty[32];
        }

        if ( (value & 0xc0) == 0xc0 )
        { // trigger && enable 
          if ( (nrx4.io & 0x40) != 0 ) 
          { // trigger && enable (open to open) 
            if ( len_ctr.timer == 0 )
            { // un-freezes enabled length should clock it (if first-half period) 
              len_ctr.timer = 64 - (apu::frame_sequencer & 1);
            }
          }
          else if ( (apu::frame_sequencer & 1) != 0 && len_ctr.timer <= 1 )
          { // trigger && enable (close to open) 
            len_ctr.timer = 63;
          }
        }
        else if ( len_ctr.timer == 0 )
        { // trigger, un-freeze length that reached zero 
          len_ctr.timer = 64;
        }

        if ( (sweep.shift | sweep.load) != 0 )
        { // internal enabled flag is set if either the sweep 
          //  period or shift are non-zero, cleared otherwise. 
          sweep.enable_mask = UINT32_MAX;
        }

        if ( sweep.shift != 0 )
        { // If the sweep shift is non-zero, 
          //  frequency calculation and the overflow 
          // check are performed immediately.
          if ( sweep::scanning (sweep) > 2047 )
          {
            squ.chan::enable_mask = 0;
          }
        }
      }
      else
      {
        if ( (~nrx4.io & value & 0x40) != 0
         && (apu::frame_sequencer & 1) != 0
          &&  len_ctr.timer != 0 )
        { // extra length clocking occurs when writing to NRx4 when the 
          //  frame sequencer's next step is one that doesn't clock the length counter. 
          // In this case, if the length counter was PREVIOUSLY disabled and now enabled 
          // and the length counter is not zero, it is decremented. 
          // If this decrement makes it zero and trigger is clear, the channel is disabled
          if ( --len_ctr.timer == 0 )
          { // disable channel  
            squ.chan::enable_mask = 0;
            squ.duty = & apu::duty[32];
          }
        }
        len_ctr.enable_mask = 
            (value & 0x40) ? UINT32_MAX : 0;
      }
      nrx4.io = value;
      nrx4.io&= ~7;
      nrx4.io|= freq >> 8 & 7;
      nrx3.io = static_cast<uint8_t> (freq);
    }
  }

  IR_FORCEINLINE void 
  nr30_write (uint8_t value )
  { // Bit 7 - Sound Channel 3 Off (0=Stop, 1=Playback)
    if ( apu::enable_mask != 0 )
    {
      wav_chan &wav = wav_chan_;
      reg &reg = wav.regs[0];

      wav.chan::dac_mask = UINT32_MAX;

      if ( static_cast<int8_t>( value) >= 0 )
      { // close DAC
        wav.chan::dac_mask = 0;
        wav.chan::sft = 4;
        wav.chan::enable_mask = 0;
      }
      reg.io = value;
    }
  }

  IR_FORCEINLINE void 
  nr31_write (uint8_t value )
  {
    if ( apu::enable_mask != 0 )
    { // Bit 7-0 - Sound length (t1: 0 - 255)
      // Sound Length = (256-t1)*(1/256) seconds This value is used only if Bit 6 in NR34 is set. 
      wav_chan &wav = wav_chan_;

      len_ctr &len_ctr = wav;
      reg &reg = wav.regs[1];

      len_ctr.timer = 256 - value;

      reg.io = value;  
    }
  }

  IR_FORCEINLINE void 
  nr32_write (uint8_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // Bit 6-5 - Select output level (Read/Write)
      //   Possible Output levels are:
      // 
      // 0: Mute (No sound)
      // 1: 100% Volume (Produce Wave Pattern RAM Data as it is)
      // 2:  50% Volume (Produce Wave Pattern RAM data shifted once to the right)
      // 3:  25% Volume (Produce Wave Pattern RAM data shifted twice to the right) 
      wav_chan &wav = wav_chan_;

      len_ctr &len_ctr = wav;
      reg &reg = wav.regs[2];

      wav.sft = ((value >> 5) & 3) - 1;

      if ( (wav.enable_mask & ((value >> 5) & 3)) == 0  )
      {
        wav.sft = 4;
      }
      reg.io = value;  
    }
  }

  IR_FORCEINLINE void 
  nr33_write (uint8_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // Lower 8 bits of 11 bit frequency (x)
      wav_chan &wav = wav_chan_;

      reg &nrx3 = wav.regs[3];
      reg &nrx4 = wav.regs[4];

      uint32_t freq = nrx4.io & 7;
               freq<<= 8;
               freq |= value;

      // channel freq:2097152 / (2048 - freq_11bit) 
      // range:2097152~1024 hz (65536~32 hz for tone)
      // master:4194304 hz := 2097152 * 32 ~ 1024 * 4096
      // 
      //            2097152                  2048 - f
      // 4194304 / ----------- => 4194304 * ----------- => (2048 - f) * 2
      //            2048 - f                 2097152
      wav.chan::load = (2048 - freq) << 1;

      nrx3.io = value;  
    }
  }

  IR_FORCEINLINE void 
  nr34_write (uint8_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // Bit 7   - Initial (1=Restart Sound)     (Write Only)
      // Bit 6   - Counter/consecutive selection (Read/Write)
      //           (1=Stop output when length in NR31 expires)
      // Bit 2-0 - Frequency's higher 3 bits (x) (Write Only) 
      wav_chan &wav = wav_chan_;

      len_ctr &len_ctr = wav_chan_;

      reg &nrx0 = wav.regs[0];
      reg &nrx1 = wav.regs[1];
      reg &nrx2 = wav.regs[2];
      reg &nrx3 = wav.regs[3];
      reg &nrx4 = wav.regs[4];

      uint32_t freq = value & 7;
               freq<<= 8;
               freq|= nrx3.io;

      wav.chan::load = (2048 - freq) << 1;

      if ( static_cast <int8_t> (value) < 0 )
      { // Writing a value to NR34 with bit 7 set causes the following things to occur:
        // 
        // Channel is enabled (see length counter).
        // If length counter is zero, it is set to 256.
        // Frequency timer is reloaded with period.
        // Wave channel's position is set to 0 but sample buffer is NOT refilled.
        // Note that if the channel's DAC is off, 
        // after the above actions occur the channel will be immediately disabled again.
        if ( (value & 0xc0) == 0xc0 )
        { // trigger && enable 
          if ( (nrx4.io & 0x40) != 0 ) 
          { // trigger && enable (open to open) 
            if ( len_ctr.timer == 0 )
            { // un-freezes enabled length should clock it (if first-half period) 
              len_ctr.timer = 256 - (apu::frame_sequencer & 1);
            }
          }
          else if ( (apu::frame_sequencer & 1) != 0 && len_ctr.timer <= 1 )
          { // trigger && enable (close to open)
            len_ctr.timer = 255;
          }
        }
        else if ( len_ctr.timer == 0 )
        { // trigger, un-freeze length that reached zero
          len_ctr.timer = 256;
        }
        wav.enable_mask = wav.dac_mask;
        wav.timer = wav.load;
        wav.pos = 0;
        wav.sft = ((nrx2.io >> 5) & 3) - 1;

        if ( (wav.enable_mask & ((nrx2.io >> 5) & 3)) == 0  )
        {
          wav.sft = 4;
        }
        len_ctr.enable_mask = 
          (value & 0x40) != 0 ? UINT32_MAX : 0;
      }
      else
      {
        if ( (~nrx4.io & value & 0x40) != 0
         && (apu::frame_sequencer & 1) != 0
          &&  len_ctr.timer != 0 )
        { // extra length clocking occurs when writing to NRx4 when the 
          //  frame sequencer's next step is one that doesn't clock the length counter. 
          // In this case, if the length counter was PREVIOUSLY disabled and now enabled 
          // and the length counter is not zero, it is decremented. 
          // If this decrement makes it zero and trigger is clear, the channel is disabled
          if ( --len_ctr.timer == 0 )
          { // disable channel 
            wav.enable_mask = 0;
            wav.sft = 8;
          }
        }
        len_ctr.enable_mask = 
            (value & 0x40) ? UINT32_MAX : 0;
      }
      nrx4.io = value;
    }  
  }

  IR_FORCEINLINE void 
  wav_ram_write (uint16_t index, uint8_t value )
  { // This storage area holds 32 4-bit samples that are played back, upper 4 bits first.
    // Wave RAM should only be accessed while CH3 is disabled (NR30 bit 7 reset), 
    // otherwise accesses will behave weirdly.
    // 
    // On almost all models, the byte will be written at the offset CH3 is currently reading. 
    // On GBA, the write will simply be ignored.
    wav_chan &wav = wav_chan_;

    if ( wav.enable_mask != 0 )
    {
      index = wav.pos - 1;
      index&= 30;
      index>>= 1;
    }
    index &= 15;
    index<<= 1;

    wav.table[index + 0] = value >> 4;
    wav.table[index + 1] = value & 15;
  }

  IR_FORCEINLINE uint8_t 
  wav_ram_read (uint16_t index )
  { 
    wav_chan &wav = wav_chan_;

    if ( wav.enable_mask != 0 )
    {
      index = wav.pos - 1;
      index&= 30;
      index>>= 1;
    }
    index &= 15;
    index<<= 1;

    return wav.table[index + 0] << 4
          | wav.table[index + 1];
  }

  IR_FORCEINLINE void 
  nr41_write (uint8_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // Bit 5-0 - Sound length data (t1: 0-63)
      // Sound Length = (64-t1)*(1/256) seconds The Length value is used only if Bit 6 in NR44 is set.
      noi_chan &noi = noi_chan_;

      len_ctr &len_ctr = noi;
      reg &reg = noi.regs[1];

      len_ctr.timer = 64 - (value & 63);

      reg.io = value; 
    }
  }

  IR_FORCEINLINE void 
  nr42_write (uint8_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // Bit 7-4 - Initial Volume of envelope (0-0Fh) (0=No Sound)
      // Bit 3   - Envelope Direction (0=Decrease, 1=Increase)
      // Bit 2-0 - Number of envelope sweep (n: 0-7)
      // (If zero, stop envelope operation.)
      noi_chan &noi = noi_chan_;

      len_ctr &len_ctr = noi;
      reg &reg = noi.regs[2];

      noi.chan::dac_mask = UINT32_MAX;

      if ( (value & 0xf8) == 0 )
      { // close channel's DAC
        noi.chan::enable_mask = 0;
        noi.dac_mask = 0;
      }
      reg.io = value; 
    }  
  }

  IR_FORCEINLINE void 
  nr43_write (uint8_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // The amplitude is randomly switched between high and low at the given frequency. 
      // A higher frequency will make the noise to appear 'softer'. 
      // When Bit 3 is set, the output will become more regular, 
      // and some frequencies will sound more like Tone than Noise.
      // 
      // Bit 7-4 - Shift Clock Frequency (s)
      // Bit 3   - Counter Step/Width (0=15 bits, 1=7 bits)
      // Bit 2-0 - Dividing Ratio of Frequencies (r)  
      // channel freq:262144 / (r * 2^n) 
      // 
      // master:4194304 hz
      // 
      //            262144                  r * (2^s)
      // 4194304 / ----------- => 4194304 * ----------- => (r * (2^s)) * 16 ( r = 0.5 if r is 0 )
      //            r * (2^s)                 262144
      noi_chan &noi = noi_chan_;

      reg &reg = noi.regs[3];

      uint32_t s = value >> 4;

      noi.chan::load = divisor[value & 7] << s;
      reg.io = value; 
    } 
  }

  IR_FORCEINLINE void 
  nr44_write (uint8_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // Bit 7   - Initial (1=Restart Sound)     (Write Only)
      // Bit 6   - Counter/consecutive selection (Read/Write)
      //             (1=Stop output when length in NR41 expires)
      noi_chan &noi = noi_chan_;

      len_ctr &len_ctr = noi;
      envelope &envelope = noi;

    //reg &nrx0 = noi.regs[0];
      reg &nrx1 = noi.regs[1];
      reg &nrx2 = noi.regs[2];
      reg &nrx3 = noi.regs[3];
      reg &nrx4 = noi.regs[4];

      if ( static_cast<int8_t> ( value ) < 0 )
      { // Writing a value to NR44 with bit 7 set causes the following things to occur:
        // 
        // Channel is enabled .
        // If length counter is zero, it is set to 64.
        // Frequency timer is reloaded with period.
        // Volume envelope timer is reloaded with period.
        // Channel volume is reloaded from NRx2.
        // Noise channel's LFSR bits are all set to 1.
        // 
        // Note that if the channel's DAC is off, after the above actions occur the channel will be immediately disabled again.
        if ( (value & 0xc0) == 0xc0 )
        { // trigger && enable 
          if ( (nrx4.io & 0x40) != 0 ) 
          { // trigger && enable (open to open) 
            if ( len_ctr.timer == 0 )
            { // un-freezes enabled length should clock it (if first-half period) 
              len_ctr.timer = 64 - (apu::frame_sequencer & 1);
            }
          }
          else if ( (apu::frame_sequencer & 1) != 0 && len_ctr.timer <= 1 )
          { // trigger && enable (close to open) 
            len_ctr.timer = 63;
          }
        }
        else if ( len_ctr.timer == 0 )
        { // trigger, un-freeze length that reached zero 
          len_ctr.timer = 64;
        }
        envelope.load = nrx2.io & 7;
        envelope.load_8x = (envelope.load == 0) ? 8 : envelope.load;
        envelope.timer = envelope.load_8x;
        envelope.vol = nrx2.io >> 4;
        envelope.vec = ((nrx2.io & 8) != 0) ? 0x01 : UINT8_MAX;
        envelope.enable_mask = envelope.load != 0 ? UINT32_MAX : 0;

        noi.chan::enable_mask = noi.chan::dac_mask;
        noi.chan::timer = noi.chan::load;

        noi.xor_2bit = & noi.xor_tab[nrx3.io >> 1 & 4];
        noi.lfsr = (nrx3.io & 8) == 0 ? 0x7fff : 0x7f;

        len_ctr.enable_mask = 
            (value & 0x40) ? UINT32_MAX : 0;
      }
      else
      {
        if ( (~nrx4.io & value & 0x40) != 0
         && (apu::frame_sequencer & 1) != 0
          &&  len_ctr.timer != 0 )
        { // extra length clocking occurs when writing to NRx4 when the 
          //  frame sequencer's next step is one that doesn't clock the length counter. 
          // In this case, if the length counter was PREVIOUSLY disabled and now enabled 
          // and the length counter is not zero, it is decremented. 
          // If this decrement makes it zero and trigger is clear, the channel is disabled
          if ( --len_ctr.timer == 0 )
          { // disable channel 
            noi.chan::enable_mask = 0;
          }
        }
        len_ctr.enable_mask = 
            (value & 0x40) ? UINT32_MAX : 0;
      }
      nrx4.io = value; 
    }
  }

  IR_FORCEINLINE void 
  nr50_write (uint8_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // The volume bits specify the "Master Volume" for Left/Right sound output. 
      //  SO2 goes to the left headphone, and SO1 goes to the right.
      // 
      //  Bit 7   - Output Vin to SO2 terminal (1=Enable)
      //  Bit 6-4 - SO2 output level (volume)  (0-7)
      //  Bit 3   - Output Vin to SO1 terminal (1=Enable)
      //  Bit 2-0 - SO1 output level (volume)  (0-7)
      // 
      // The Vin signal is an analog signal received from the game cartridge bus, 
      //  allowing external hardware in the cartridge to supply a fifth sound channel, 
      // additionally to the Game Boy's internal four channels. 
      //  No licensed games used this feature, and it was omitted from the Game Boy Advance.
      // 
      // (Despite rumors, Pocket Music does not use Vin. It blocks use on the GBA for a different reason: 
      //   the developer couldn't figure out how to silence buzzing associated with the wave channel's DAC.)
      nr50.io = value;
    }  
  }

  IR_FORCEINLINE void 
  nr51_write (uint8_t value )
  { 
    if ( apu::enable_mask != 0 )
    { // FF25 - NR51 - Selection of Sound output terminal (R/W)
      // Each channel can be panned hard left, center, or hard right.
      // 
      //  Bit 7 - Output sound 4 to SO2 terminal
      //  Bit 6 - Output sound 3 to SO2 terminal
      //  Bit 5 - Output sound 2 to SO2 terminal
      //  Bit 4 - Output sound 1 to SO2 terminal
      //  Bit 3 - Output sound 4 to SO1 terminal
      //  Bit 2 - Output sound 3 to SO1 terminal
      //  Bit 1 - Output sound 2 to SO1 terminal
      //  Bit 0 - Output sound 1 to SO1 terminal 
      nr51.io = value;

      value = value & (static_cast <int8_t> (nr52.io) >> 7);

      squ_chans[0].s0_mask = (value & 1) ? UINT16_MAX : 0;
      squ_chans[1].s0_mask = (value & 2) ? UINT16_MAX : 0;
      wav_chan_.s0_mask = (value & 4) ? UINT16_MAX : 0;
      noi_chan_.s0_mask = (value & 8) ? UINT16_MAX : 0;
      squ_chans[0].s1_mask = (value & 16) ? UINT16_MAX : 0;
      squ_chans[1].s1_mask = (value & 32) ? UINT16_MAX : 0;
      wav_chan_.s1_mask = (value & 64) ? UINT16_MAX : 0;
      noi_chan_.s1_mask = (value & 128) ? UINT16_MAX : 0;
    }
  }

  IR_FORCEINLINE void 
  nr52_write (uint8_t value )
  { // FF26 - NR52 - Sound on/off
    // If your GB programs don't use sound then write 00h to this 
    // register to save 16% or more on GB power consumption. 
    // Disabeling the sound controller by clearing Bit 7 destroys the contents 
    // of all sound registers. Also, it is not possible to access 
    // any sound registers (execpt FF26) while the sound controller is disabled.
    // 
    //  Bit 7 - All sound on/off  (0: stop all sound circuits) (Read/Write)
    //  Bit 3 - Sound 4 ON flag (Read Only)
    //  Bit 2 - Sound 3 ON flag (Read Only)
    //  Bit 1 - Sound 2 ON flag (Read Only)
    //  Bit 0 - Sound 1 ON flag (Read Only) 
    squ_chan &squ = squ_chans[0];
    squ_chan &squ2= squ_chans[1];
    wav_chan &wav = wav_chan_;
    noi_chan &noi = noi_chan_;

    if ( static_cast <int8_t> (apu::nr52.io ^ value) < 0 )
    { // NR52 state change 
      if ( static_cast <int8_t> (value) < 0 )
      { // close to open 
        apu::frame_counter = 0;
        apu::frame_sequencer = 0; 

        apu::enable_mask = UINT32_MAX;
      }
      else
      { // open to close 
        apu::enable_mask = 0;

        squ.len_ctr::enable_mask = 0;
        squ2.len_ctr::enable_mask = 0;
        wav.len_ctr::enable_mask = 0;
        noi.len_ctr::enable_mask = 0;

        squ.envelope::enable_mask = 0;
        squ2.envelope::enable_mask = 0;
        noi.envelope::enable_mask = 0;

        squ.chan::enable_mask = 0;
        squ2.chan::enable_mask = 0;
        wav.chan::enable_mask = 0;
        noi.chan::enable_mask = 0;

        squ.regs[0].io = 0x00;
        squ.regs[1].io = 0x00;
        squ.regs[2].io = 0x00;
        squ.regs[3].io = 0x00;
        squ.regs[4].io = 0x00;

        squ2.regs[0].io = 0x00;
        squ2.regs[1].io = 0x00;
        squ2.regs[2].io = 0x00;
        squ2.regs[3].io = 0x00;
        squ2.regs[4].io = 0x00;

        wav.regs[0].io = 0x00;
        wav.regs[1].io = 0x00;
        wav.regs[2].io = 0x00;
        wav.regs[3].io = 0x00;
        wav.regs[4].io = 0x00;

        noi.regs[0].io = 0x00;
        noi.regs[1].io = 0x00;
        noi.regs[2].io = 0x00;
        noi.regs[3].io = 0x00;
        noi.regs[4].io = 0x00;

        apu::nr50.io = 0;
        apu::nr51.io = 0;

        apu::enable_mask = 0;
      }
    }
    apu::nr52.io = value;

    value = apu::nr51.io & (static_cast <int8_t> (value) >> 7);

    squ.s0_mask = (value & 1) ? UINT16_MAX : 0;
    squ2.s0_mask = (value & 2) ? UINT16_MAX : 0;
    wav.s0_mask = (value & 4) ? UINT16_MAX : 0;
    noi.s0_mask = (value & 8) ? UINT16_MAX : 0;
    squ.s1_mask = (value & 16) ? UINT16_MAX : 0;
    squ2.s1_mask = (value & 32) ? UINT16_MAX : 0;
    wav.s1_mask = (value & 64) ? UINT16_MAX : 0;
    noi.s1_mask = (value & 128) ? UINT16_MAX : 0;
  }

  IR_FORCEINLINE void 
  sweep_tick (void)
  {
    squ_chan &squ = squ_chans[0];

    len_ctr &len_ctr = squ;
    sweep &sweep = squ;
    envelope &envelope = squ;

  //reg &nrx0 = squ.regs[0];
  //reg &nrx1 = squ.regs[1];
  //reg &nrx2 = squ.regs[2];
    reg &nrx3 = squ.regs[3];
    reg &nrx4 = squ.regs[4];

    if ( (sweep.enable_mask & sweep.timer) != 0 && --sweep.timer == 0 )
    { // update frequency 
      if ( sweep.load != 0 )
      {
        uint32_t freq = sweep::scanning (sweep);

        if ( freq > 2047 )
        {
          squ.chan::enable_mask = 0;
        }
        else if ( sweep.shift != 0 )
        {
          sweep.freq = freq;

          squ.chan::load = (2048 - freq) << 2;

          nrx3.io = freq & 0xff;
          nrx4.io&= ~7;
          nrx4.io|= freq >> 8 & 7;

          if ( sweep::scanning (sweep) > 2047 )
          { // again 
            squ.chan::enable_mask = 0;
          }
        }
      }
      sweep.timer = sweep.load_8x;
    }
  }
 
  IR_FORCEINLINE void 
  len_ctr_tick (chan &chan)
  {
    len_ctr &len_ctr = chan;

    if ( (len_ctr.enable_mask & len_ctr.timer) != 0 && --len_ctr.timer == 0 )
    { // disable channel.
      chan.enable_mask = 0; // common disable  
      chan.duty = & apu::duty[32]; // disable for squ, squ2
      chan.sft = 4; // disable for wav 
    }
  }

  IR_FORCEINLINE void 
  envelope_tick (envelope &envelope)
  {
    if ( (envelope.enable_mask & envelope.timer) != 0 && --envelope.timer == 0)  
    {           
      envelope.timer = envelope.load_8x;                 
      envelope.vol += envelope.vec;                                         
      envelope.vol -= static_cast<int8_t> (envelope.vol) >> 4;           
    }
  }

  IR_FORCEINLINE
  void frame_sequencer_tick (int32_t clk)
  {
    apu::frame_counter -= clk;

    if ( apu::frame_counter <= 0 )
    {
      apu::frame_counter += 8192;                    
                                                          
      switch (apu::frame_sequencer++)                     
      {                                                      
      case 2:                                                
      case 6:                                                
        sweep_tick ();           
      case 0:                                                
      case 4:     
        len_ctr_tick (squ_chans[0]);
        len_ctr_tick (squ_chans[1]);
        len_ctr_tick (wav_chan_);
        len_ctr_tick (noi_chan_);   
      case 1:                                                
      case 3:                                                
      case 5:                                                
        break;                                               
                                                          
      case 7:                                                
        envelope_tick (squ_chans[0]);           
        envelope_tick (squ_chans[1]);           
        envelope_tick (noi_chan_);         

        apu::frame_sequencer = 0;
        break;                                               
                                                          
      default:                                               
        IASSERT (0);                                         
        break;                                               
      } 
    }                                                
  }

  IR_FORCEINLINE
  void clk (int32_t clk)
  {
    T *gb = static_cast<T *> (this);

    if ( clk >= apu::sample_counter )
    { // synthetic sampling is required
      clk -= apu::sample_counter;

      frame_sequencer_tick (apu::sample_counter);

      squ_chans[0].tick (apu::sample_counter);
      squ_chans[1].tick (apu::sample_counter);
      wav_chan_.tick (apu::sample_counter);
      noi_chan_.tick (apu::sample_counter);

      apu::sample_counter = apu::sample_reload;

      uint32_t c0 = squ_chans[0].get_output ();
      uint32_t c1 = squ_chans[1].get_output ();
      uint32_t c2 = wav_chan_.get_output ();
      uint32_t c3 = noi_chan_.get_output ();

      gb->os_snd_synthesis ( c0 & squ_chans[0].s0_mask
                          , c1 & squ_chans[1].s0_mask
                           , c2 & wav_chan_.s0_mask
                          ,  c3 & noi_chan_.s0_mask
                           ,  c0 & squ_chans[0].s1_mask
                          , c1 & squ_chans[1].s1_mask
                           , c2 & wav_chan_.s1_mask
                          ,  c3 & noi_chan_.s1_mask );
      if ( clk == 0 )
      {
        goto exit__;
      }
    }
    frame_sequencer_tick (clk);

    squ_chans[0].tick (clk);
    squ_chans[1].tick (clk);
    wav_chan_.tick (clk);
    noi_chan_.tick (clk);

    apu::sample_counter -= clk;
exit__:;    
  }

  IR_FORCEINLINE
  void reset (void)
  { 
    squ_chans[0].reset ();
    squ_chans[1].reset ();
    wav_chan_.reset ();
    noi_chan_.reset ();

    enable_mask = 0;

    nr50.io = 
    nr51.io = 
    nr52.io = 0;

    nr50_write ( 0x77 );
    nr51_write ( 0xf3 );
    nr52_write ( 0xf1 ); 

    frame_counter = 0;
    frame_sequencer = 0;

    if ( sample_reload == 0 )
    {
      sample_reload = 4194304 / 44100;
      sample_counter = sample_reload;
    }
  }
  squ_chan squ_chans[2];
  wav_chan wav_chan_;
  noi_chan noi_chan_;

  uint32_t enable_mask; // apu master enable mask 

  uint8_t duty[40];
  uint32_t divisor[8];

  int32_t frame_counter; 
  int32_t frame_sequencer; 

  int32_t sample_counter;
  int32_t sample_reload;

  reg nr50;
  reg nr51;
  reg nr52;
};

template <class T>
struct ppu
{
  struct tile_map;

  ppu (void)
  {
    uint32_t *gs_pal_ = reinterpret_cast <uint32_t *> ( & gs_pal[0] );

    pal_tmr_[0].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 0]);
    pal_tmr_[1].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 1]);
    pal_tmr_[2].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 2]);
    pal_tmr_[3].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 3]);
    pal_tmr_[4].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 4]);
    pal_tmr_[5].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 5]);
    pal_tmr_[6].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 6]);
    pal_tmr_[7].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 7]);
    pal_tmr_[8].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 8]);
    pal_tmr_[9].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 9]);
    pal_tmr_[10].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 10]);
    pal_tmr_[11].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 11]);
    pal_tmr_[12].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 12]);
    pal_tmr_[13].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 13]);
    pal_tmr_[14].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 14]);
    pal_tmr_[15].record = reinterpret_cast <tile_map **> (& ppu::prec_mem[768 * 15]);

#if 1
    transition_gs_pal (0, 0x00ffffff, 0x6b7b73, 0x393931, 0x000000);
    transition_gs_pal (1, 0x00ffffff, 0x227b52, 0x545104, 0x00ff00);
    transition_gs_pal (2, 0x00ffffff, 0xcece84, 0x393931, 0x39397b);
    transition_gs_pal (3, 0x00ffffff, 0xb57300, 0x393931, 0x001010);
#else 
    transition_gs_pal (0, 0x00ffffff, 0xaaaaaa, 0x555555, 0x000000);
    transition_gs_pal (1, 0x00ffffff, 0xaaaaaa, 0x555555, 0x000000);
    transition_gs_pal (2, 0x00ffffff, 0xaaaaaa, 0x555555, 0x000000);
    transition_gs_pal (3, 0x00ffffff, 0xaaaaaa, 0x555555, 0x000000);
#endif 
    bcp_ocp_[0].rec = & pal_tmr_[0];
    bcp_ocp_[1].rec = & pal_tmr_[8];

    bcp_ocp_[0].set_mask = 0x80000000;
    bcp_ocp_[0].reset_mask = 0x02000000;
    bcp_ocp_[1].set_mask = 0x80000000;
    bcp_ocp_[1].reset_mask = 0x01000000;

#define GRAPHICS_BUILD_(x)                                                    \
  graphics_renders[x] = & ppu<T>::render_graphics< (x & 16) ? UINT32_MAX : 0  \
                                              , (x & 8) ? UINT32_MAX : 0      \
                                               ,  (x & 4) ? UINT32_MAX : 0    \
                                              , (x & 2) ? UINT32_MAX : 0      \
                                               , (x & 1) ? UINT32_MAX : 0 >;
    GRAPHICS_BUILD_(0)
    GRAPHICS_BUILD_(1)
    GRAPHICS_BUILD_(2)
    GRAPHICS_BUILD_(3)
    GRAPHICS_BUILD_(4)
    GRAPHICS_BUILD_(5)
    GRAPHICS_BUILD_(6)
    GRAPHICS_BUILD_(7)
    GRAPHICS_BUILD_(8)
    GRAPHICS_BUILD_(9)
    GRAPHICS_BUILD_(10)
    GRAPHICS_BUILD_(11)
    GRAPHICS_BUILD_(12)
    GRAPHICS_BUILD_(13)
    GRAPHICS_BUILD_(14)
    GRAPHICS_BUILD_(15)
    GRAPHICS_BUILD_(16)
    GRAPHICS_BUILD_(17)
    GRAPHICS_BUILD_(18)
    GRAPHICS_BUILD_(19)
    GRAPHICS_BUILD_(20)
    GRAPHICS_BUILD_(21)
    GRAPHICS_BUILD_(22)
    GRAPHICS_BUILD_(23)
    GRAPHICS_BUILD_(24)
    GRAPHICS_BUILD_(25)
    GRAPHICS_BUILD_(26)
    GRAPHICS_BUILD_(27)
    GRAPHICS_BUILD_(28)
    GRAPHICS_BUILD_(29)
    GRAPHICS_BUILD_(30)
    GRAPHICS_BUILD_(31)
  }

  void transition_gs_pal ( uint32_t slot, uint32_t p0, uint32_t p1, uint32_t p2, uint32_t p3 )
  {
    uint32_t *apl = reinterpret_cast<uint32_t *> ( & gs_pal[slot << 4] );

    apl[0] = p0;
    apl[1] = p1;
    apl[2] = p2;
    apl[3] = p3;
  }

  union nt_rdc
  { // nametable render describe chunk 
    struct
    {
      uint8_t filp_x; // 0xff or 0 (for cgb mode 
      uint8_t filp_y_7l5x; // 0x07 << 5 or 4 or 0 (for cgb mode) 
      uint8_t id; // tile id 
      uint8_t attributes; // for cgb mode 

      uint32_t pri;    // 0x00000000 or 0x01000000  (for cgb mode) 

      tile_map *entry[4]; // with tile-id, pal index, bank (bg/win for dmg)
  #ifdef _DEBUG
      uint8_t pid; // page id 
      uint8_t x;
      uint8_t y;
  #endif 
    };
  #ifdef _DEBUG
    uint8_t unused[sizeof (void *) << 3];
  #else 
    uint8_t unused[sizeof (void *) << 2];
  #endif 
  };
  
  struct pal_tmr
  { // palette used by the target machine for rendering 
    uint8_t session[16]; // gb bg:tran  | 0x00000000
                         // gb bg:solid | 0x80000000
                         // gb sp:solid | 0x80000000
                         // gb sp:tran  | 0x00000000
                         // cgb bg:tran  | 0x02000000
                         // cgb bg:solid | 0x80000000
                         // cgb sp:solid | 0x80000000
                         // cgb sp:tran  | 0x01000000
         uintptr_t elements;
    tile_map **record; // tile map record pool
  };

  struct tile_map
  {
    uint8_t *pix;
  #ifdef _DEBUG 
    union
    {
      struct 
      { 
        uintptr_t mid[3]; // 0:0x8000 mode (unsigned address) 
                          // 1:0x8800 mode (signed address) 
                          // 2:full address mode 
        uint16_t pid; // page id always 0 for gb 
        uintptr_t sid; // for palette index 
      };
      uint8_t align[sizeof (void *) * 4];
    };
    uint32_t epos; // 
  #endif 
    nbyte *chr;

    tile_map **entry; // position of pal_tmr record
    pal_tmr *pal;
  };

  struct tile_pixel
  {
    uint8_t data[384 * 256 * 8 * 2 + 64];
    uint8_t *ptr;
    uint8_t *rec[384 * 8 * 2];
    uint32_t recs;
    uint32_t cnts; // alloc counts (for debug)
  };

  union sprite
  {
    struct
    {
      uint8_t y;
      uint8_t x;
      uint8_t attributes;
      uint8_t id;

      tile_map *entry; // with tile-id, pal index, bank, & 0xFE, & UINT8_MAX

      uint8_t odd_mask; // 16(cgb)/4(gb) or 0 
      uint8_t filp_x; // 0xff or 0 
      uint8_t filp_y_7l5x; // 0x07 << 5 or 4 or 0 
      uint16_t filp_y_15l5x; // 0x0F << 5 or 4 or 0 
      uint8_t pri;    // 0xff or 0 

      uint16_t y2nx; // combine 
    };
    uint8_t align[sizeof (void *) << 2];
  };

  struct hdma
  {
    uint32_t src_lo;
    uint32_t src_hi;
    uint32_t dst_lo;
    uint32_t dst_hi;
    uint32_t control;
  };

  struct bcp_ocp
  {
    uint32_t access; 
    uint32_t increment; 
    uint32_t source; // bcps or ocps 
    uint32_t set_mask;
    uint32_t reset_mask;

    uint8_t pal[64];  

    pal_tmr *rec; // bgp or sp entry 
  };

  uint8_t control;
  uint8_t stat;
  uint16_t line;
  uint8_t ly_cmp;
  uint32_t win_y;
  uint32_t win_x;
  uint32_t scr_y;
  uint32_t scr_x;
  uint8_t vbk;
  uint8_t oam_dma;
  uint8_t bgp_obp[4];

  uint32_t m0_sel_mask; // mode 0 h-blank 
  uint32_t m1_sel_mask; // mode 1 v-blank 
  uint32_t m2_sel_mask; // mode 2 oam scan 
  uint32_t lyc_sel_mask;
  uint32_t ly_equal_mask; // ! >!
  uint32_t mode; // ! >!

  uint8_t *scanline; // scanline,  margin 8 * 2 pixel
  uintptr_t scanline_pitch; // pitch 

  tile_map tile_map_[384 * 2 * 8 * 2];
  nt_rdc nt_rdc_[32 * 32 * 2];
  nt_rdc *bg_nametable;
  nt_rdc *win_nametable;
  pal_tmr pal_tmr_[16]; // 0~7:background 8~15:sprite 
  sprite sprite_[40];
  tile_pixel tile_data_;
  bcp_ocp bcp_ocp_[2];
  hdma dma_;
  nbyte vram[MEM_8K * 2];

  intptr_t clk;

  uint8_t gs_pal[64]; // gb gray-scale palette mapper 
                      // d0:bg  d1:win  d2:obp0 d3:obp1

  uint8_t **prec_mem[384 * 2 * 8 * 2];

  uint32_t data_slot; // 0 for 0x8000, 1 for 0x8800 
  uint32_t win_counter; 

  uint8_t graphics_mask; // d4:cgb render mask
                         // d3:lcd pri mask 
                         // d2:win enable mask 
                         // d1:sprite enable mask 
                         // d0:sprite size 16 mask
  void (ppu<T>::*graphics_render)(void);
  void (ppu<T>::*graphics_renders[32])(void);

  template <uint32_t t_nCgbMask 
        , uint32_t t_nLcdPriMask   
         , uint32_t t_nWinEnableMask 
          , uint32_t t_nSpriteEnableMask
         , uint32_t t_nSprite16Mask>
  IR_NOINLINE
  void render_graphics (void)
  {
    struct helper
    {
      IR_STATIC_FORCEINLINE
      __m256i _mm256_reverse_epi32 ( __m256i v )
      {
        return  _mm256_permute4x64_epi64 ( _mm256_shuffle_epi32 (v, 0x1B), 0x4E);
      }

      IR_STATIC_FORCEINLINE
      void bg_pixel_generate (  _in_ const nt_rdc *const nd
                             , _in_ const tile_map *const tm
                            , _in_      uint8_t *tile
                             , _inout_ uint8_t *scanline
                            , _in_ const __m256i trans )
      {
        __m256i bg_mixer = _mm256_load_si256 (reinterpret_cast<const __m256i *> (tile));
        __m256i bg_lock;
        __m256i bg_sel;
        __m256i res_pix;
     
        if ( t_nCgbMask != 0 && nd->filp_x != 0 )
        {
          bg_mixer = _mm256_reverse_epi32 (bg_mixer);
        }

        if ( t_nCgbMask != 0 )
        { // cgb render mode
          if ( t_nLcdPriMask != 0 )
          { // cgb lock mode 
            bg_lock = _mm256_set1_epi32 (nd->pri);                
            bg_sel = _mm256_srai_epi32 (bg_mixer, 31);                       
            bg_mixer= _mm256_or_si256 (bg_mixer, bg_lock);                                       
          }
          else
          { // cgb free mode          
            bg_sel = _mm256_srai_epi32 (bg_mixer, 31);    
          }
        }
        else
        { // dmg render mode
          bg_sel = _mm256_srai_epi32 (bg_mixer, 31);              
        }
        res_pix = _mm256_blendv_epi8 (trans, bg_mixer, bg_sel);  

        _mm256_storeu_si256 (reinterpret_cast<__m256i *> (scanline), res_pix); 
      }

      IR_STATIC_FORCEINLINE
      void sp_pixel_generate ( _in_ const sprite *sp
                             , _in_ const tile_map *const tm
                            , _in_      uint8_t *tile
                             , _inout_ uint8_t *scanline )
      {
        __m256i bg_mixer = _mm256_loadu_si256 (reinterpret_cast<const __m256i *> (scanline));
        __m256i sp_pix = _mm256_load_si256 (reinterpret_cast<const __m256i *> (tile));
        __m256i sp_sel;
        __m256i bg_sel;
        __m256i res_pix;

        if ( sp->filp_x != 0 )
        {
          sp_pix = _mm256_reverse_epi32 (sp_pix);
        }

        if ( t_nCgbMask != 0 )
        { // cgb render mode
          if ( t_nLcdPriMask != 0 )
          { // cgb lock mode        
            if ( sp->pri != 0 )
            { // background sprite.
              sp_sel = _mm256_andnot_si256 (bg_mixer, sp_pix);                             
              sp_sel = _mm256_srai_epi32 (sp_sel, 31);  
              res_pix = _mm256_blendv_epi8 (bg_mixer, sp_pix, sp_sel);  
            }
            else
            { // foreground sprite.
              bg_sel = _mm256_sub_epi8 (sp_pix, bg_mixer);                              
              bg_sel = _mm256_srai_epi32 (bg_sel, 31);                                    
              res_pix = _mm256_blendv_epi8 (sp_pix, bg_mixer, bg_sel); 
            }                                        
          }
          else
          { // cgb free mode          
            if ( sp->pri != 0 )
            { // background sprite.
              sp_sel = _mm256_andnot_si256 (bg_mixer, sp_pix);                             
              sp_sel = _mm256_srai_epi32 (sp_sel, 31);  
              res_pix = _mm256_blendv_epi8 (bg_mixer, sp_pix, sp_sel); 
            }
            else
            { // foreground sprite.
              sp_sel = _mm256_srai_epi32 (sp_pix, 31);
              res_pix = _mm256_blendv_epi8 (bg_mixer, sp_pix, sp_sel); 
            }  
          }
        }
        else 
        {
          if ( sp->pri != 0 )
          { // background sprite.
            sp_sel = _mm256_andnot_si256 (bg_mixer, sp_pix);                             
            sp_sel = _mm256_srai_epi32 (sp_sel, 31);  
            res_pix = _mm256_blendv_epi8 (bg_mixer, sp_pix, sp_sel); 
          }
          else
          { // foreground sprite.
            sp_sel = _mm256_srai_epi32 (sp_pix, 31);
            res_pix = _mm256_blendv_epi8 (bg_mixer, sp_pix, sp_sel); 
          }
        }
        _mm256_storeu_si256 (reinterpret_cast<__m256i *> (scanline), res_pix); 
      }
    };
    uint32_t scr_y = ppu::line + ppu::scr_y;
    uint32_t scr_x = ppu::scr_x;
    uint32_t win_x = ppu::win_x;

    uint32_t mini_off_x = scr_x & 7;
    uint32_t mini_off_y_l5x = (scr_y & 7) << 5;
    uint32_t block_off_x = scr_x >> 3;
    uint32_t scans;
     
    uint32_t id;
    uint16_t y;

    uint32_t offx_t = (8 - mini_off_x) * 4;
    uint8_t *scanline_= ppu::scanline;
    uint8_t *scanline = scanline_ + offx_t;
 
    uint32_t bg_slot = ppu::data_slot;
    uint32_t win_slot= (t_nCgbMask != 0) ? bg_slot : (bg_slot + 2); // for dmg extra palette render.

    uint8_t *pix;

    nt_rdc *nt_y = & ppu::bg_nametable[(scr_y & 0xF8) << 2];

    ppu::scanline = scanline_ + ppu::scanline_pitch;

    __m256i trans;

    sprite *sp;
    tile_map *m;
    nt_rdc *n;

    if ( (t_nCgbMask | t_nLcdPriMask) != 0 )
    { // normal render (dmg mode) or cgb render mode.
      id = 21;
  
      trans = _mm256_set1_epi32 ( reinterpret_cast<int *> ( & ppu::pal_tmr_[0].session[0] )[0] );

      do
      { // render background
        n = & nt_y[block_off_x & 0x1F];
        m = n->entry[bg_slot];

        if ( m->pix == nullptr )
        { // render 8x8 gray-scale pixel
          render_tile (m);
        }

        if ( t_nCgbMask != 0 )
        {
          pix = m->pix + ( mini_off_y_l5x ^ n->filp_y_7l5x);
        }
        else
        {
          pix = m->pix + mini_off_y_l5x;
        }
        helper::bg_pixel_generate (n, m, pix, scanline, trans);

        scanline += 32;

        block_off_x += 1;
      } while (--id != 0);

      if ( t_nWinEnableMask != 0 && ppu::line >= ppu::win_y && ppu::win_x <= 166 )
      { // render window.
        scr_y = ppu::win_counter++;
        scans = (167 - ppu::win_x + 7) >> 3; 

        win_x++;
        win_x <<= 2;

        scanline = & scanline_[win_x];

        mini_off_y_l5x = (scr_y & 7) << 5;

        nt_y = & ppu::win_nametable[(scr_y & 0xF8) << 2];
         
        for ( id = 0; id != scans; id++ )
        { // render window 
          n = & nt_y[id];
          m = n->entry[win_slot];

          if ( m->pix == nullptr )
          { // render 8x8 gray-scale pixel 
            render_tile (m);
          }

          if ( t_nCgbMask != 0 )
          {
            pix = m->pix + ( mini_off_y_l5x ^ n->filp_y_7l5x);
          }
          else
          {
            pix = m->pix + mini_off_y_l5x;
          }
          helper::bg_pixel_generate (n, m, pix, scanline, trans);  

          scanline += 32;
        }
      }
    }
    else
    { // When Bit 0 is cleared, both background and window become blank (white), 
      // and the Window Display Bit is ignored in that case. 
      //  Only objects may still be displayed (if enabled in Bit 1). 
  #if 0
      trans = _mm256_set1_epi32 ( * reinterpret_cast<int *> (& ppu::pal_tmr[0].session[0] ));
  #else 
      trans = _mm256_set1_epi32 ( * reinterpret_cast<int *> (& ppu::gs_pal[0]) );
  #endif 
      id = 5;

      do
      { // render background 
        _mm256_storeu_si256 (reinterpret_cast<__m256i *> (scanline) + 0, trans);  
        _mm256_storeu_si256 (reinterpret_cast<__m256i *> (scanline) + 1, trans);
        _mm256_storeu_si256 (reinterpret_cast<__m256i *> (scanline) + 2, trans);
        _mm256_storeu_si256 (reinterpret_cast<__m256i *> (scanline) + 3, trans);

        scanline += 128;

      } while (--id != 0);
    }

    if ( t_nSpriteEnableMask != 0 )
    { // render sprite tile 
      id = 40;
      sp = & ppu::sprite_[39];

      do
      {
        if ( (y = ppu::line + sp->y2nx) <= (t_nSprite16Mask != 0 ? 15 : 7) )
        {
          if (t_nSprite16Mask != 0) 
          { // sprite 8x16
            y = y << 5 ^ sp->filp_y_15l5x;
            m = & sp->entry[ ( t_nCgbMask != 0 ) ? (y >> 4 & 16) : (y >> 6 & 4)];
                                
            if ( m->pix == nullptr )
            { // render 8x8 gray-scale pixel 
              render_tile (m);
            }
            pix = m->pix + (y & 255);

            scanline = & scanline_[sp->x << 2];
          }
          else
          { // sprite 8x8
            if ( (m = & sp->entry[sp->odd_mask])->pix == nullptr )
            { // render 8x8 gray-scale pixel 
              render_tile (m);
            }
            pix = m->pix + ( y << 5 ^ sp->filp_y_7l5x);

            scanline = & scanline_[sp->x << 2];
          }
          helper::sp_pixel_generate ( sp, m, pix, scanline);
        }
        sp--;
        id--;
      } while (id != 0);
    }
  }

  IR_NOINLINE
  void clk_internal (void)
  {   
    T *gb = static_cast<T *> (this);

    if ( (static_cast<int8_t> (ppu::control)) < 0 )
    { // lcd enable
      if ( ppu::line < 144 )
      {
        if ( ppu::mode == 3 )
        { // drawing pixels mode, horizontal blank next 
          gb->interrupt_requset (INT_STAT, ppu::m0_sel_mask); 

          if ( (gb->state & 0x8000 ) != 0 && ( ( (gb->state & 4) == 0) || (gb->mpu::ie_mask & gb->mpu::if_mask & 0x1F) != 0) )
          { // hbl-dma ? 
            ppu::hbl_dma ();      
          }
          ppu::clk += 180;
          ppu::mode = 0;
        }
        else if ( ppu::mode == 0 )
        { // horizontal blank, oam scan mode/vertical blank next 
          if ( ppu::line == 143 )
          { // mode 1 - vblank 
            gb->interrupt_requset (INT_STAT, ppu::m1_sel_mask); 
            gb->interrupt_requset (INT_VBL, UINT32_MAX); 

            ppu::clk += 456;
            ppu::mode = 1;
            ppu::line = 144;

            ppu::win_counter = 0;

            gb->joypad::update ();

            if ( ppu::scanline != nullptr )
            {
              gb->os_vid_unlock ();
              gb->os_vid_flush ();
            }
            ppu::scanline = nullptr;
          }
          else
          { // mode 2 - oam scan 
            gb->interrupt_requset (INT_STAT, ppu::m2_sel_mask); 

            ppu::clk += 80;
            ppu::mode = 2;
            ppu::line++;
          }
          ppu::compare_line ();
        }
        else if ( ppu::mode == 2 )
        { // oam scan mode, drawing pixels next 
          ppu::clk += 196;
          ppu::mode = 3;

          if ( ppu::line == 0 && ppu::scanline == nullptr )
          {
            if ( gb->os_vid_lock ( ppu::scanline, ppu::scanline_pitch ) != 0 )
            {
              ppu::scanline = nullptr;
            }
          }

          if ( ppu::scanline == nullptr )
          {
            if ( (ppu::control & 32) != 0
              && ppu::line >= ppu::win_y
                && ppu::win_x <= 166 )
            { 
              ppu::win_counter++;
            }
          }
          else 
          { 
      #ifdef IR_FAST_X86_SIMD_VECTOR
            ( (* this).*graphics_render) ();
      #else 
      #error "error"
      #endif 
          }
        }
        else
        {
          IASSERT (0);
        } 
      }
      else
      { // v-blank period 
        if ( ppu::line == 153 )
        { // to next frame 
          ppu::clk += 80;
          ppu::mode = 2;
          ppu::line = 0;

          gb->interrupt_requset (INT_STAT, ppu::m2_sel_mask); 
        }
        else
        {
          ppu::clk += 456;
          ppu::mode = 1;

          ppu::line++;
        }
        compare_line ();
      }
    }
    else
    { // lcd disable
      ppu::clk += 70224;

      gb->mpu::int_pending |= 0x40;
    }
  }

  IR_FORCEINLINE uint8_t 
  stat_read (void) const 
  { // Bit 6 - LYC=LY Coincidence Interrupt (1=Enable) (Read/Write)
    // Bit 5 - Mode 2 OAM Interrupt         (1=Enable) (Read/Write)
    // Bit 4 - Mode 1 V-Blank Interrupt     (1=Enable) (Read/Write)
    // Bit 3 - Mode 0 H-Blank Interrupt     (1=Enable) (Read/Write)
    // Bit 2 - Coincidence Flag  (0:LYC<>LY, 1:LYC=LY) (Read Only)
    // Bit 1-0 - Mode Flag       (Mode 0-3, see below) (Read Only)
    //       0: During H-Blank
    //       1: During V-Blank
    //       2: During Searching OAM
    //       3: During Transferring Data to LCD Driver
    return (ppu::stat & 0xfc) | static_cast<uint8_t> (ppu::mode);
  }

  IR_FORCEINLINE void 
  stat_write (uint8_t value) 
  { // Bit 6 - LYC=LY Coincidence Interrupt (1=Enable) (Read/Write)
    // Bit 5 - Mode 2 OAM Interrupt         (1=Enable) (Read/Write)
    // Bit 4 - Mode 1 V-Blank Interrupt     (1=Enable) (Read/Write)
    // Bit 3 - Mode 0 H-Blank Interrupt     (1=Enable) (Read/Write)
    // Bit 2 - Coincidence Flag  (0:LYC<>LY, 1:LYC=LY) (Read Only)
    // Bit 1-0 - Mode Flag       (Mode 0-3, see below) (Read Only)
    //       0: During H-Blank
    //       1: During V-Blank
    //       2: During Searching OAM
    //       3: During Transferring Data to LCD Driver
    ppu::lyc_sel_mask = (value & 0x40) != 0 ? UINT32_MAX : 0;
    ppu::m2_sel_mask = (value & 0x20) != 0 ? UINT32_MAX : 0;
    ppu::m1_sel_mask = (value & 0x10) != 0 ? UINT32_MAX : 0;
    ppu::m0_sel_mask = (value & 0x08) != 0 ? UINT32_MAX : 0;

    ppu::stat = value;
  }

  IR_FORCEINLINE void 
  scx_write ( uint8_t value) 
  { 
    ppu::scr_x = value;
  }

  IR_FORCEINLINE void 
  scy_write (uint8_t value) 
  { 
    ppu::scr_y = value;
  }

  IR_FORCEINLINE void 
  wx_write (uint8_t value) 
  { 
    ppu::win_x = value;
  }

  IR_FORCEINLINE void 
  wy_write (uint8_t value) 
  { 
    ppu::win_y = value;
  }

  IR_FORCEINLINE void 
  lcdc_write (uint8_t value) 
  { // Bit 7 - LCD Display Enable             (0=Off, 1=On)
    // Bit 6 - Window Tile Map Display Select (0=9800-9BFF, 1=9C00-9FFF)
    // Bit 5 - Window Display Enable          (0=Off, 1=On)
    // Bit 4 - BG & Window Tile Data Select   (0=8800-97FF, 1=8000-8FFF)
    // Bit 3 - BG Tile Map Display Select     (0=9800-9BFF, 1=9C00-9FFF)
    // Bit 2 - OBJ (Sprite) Size              (0=8x8, 1=8x16)
    // Bit 1 - OBJ (Sprite) Display Enable    (0=Off, 1=On)
    // Bit 0 - BG/Window Display/Priority     (0=Off, 1=On)
    T *gb = static_cast<T *> (this);

    if ( static_cast<int8_t> (ppu::control ^ value) < 0 )
    { // state change
      if ( static_cast<int8_t> (value) < 0 )
      { // close to open
        ppu::mode = 2;
        ppu::line = 0;
        ppu::clk = 80;
        ppu::compare_line ();
      }
      else
      { // open to close
        if ( gb->os_vid_islock () == 0 )
        {
          gb->os_vid_unlock (); 
        }
        gb->os_vid_flush ();

        ppu::clk = 70224;
        ppu::win_counter = 0;

        ppu::mode = 0;
        ppu::line = 0;
      }
    }
    ppu::win_nametable = (value & 0x40) != 0 
                      ? & ppu::nt_rdc_[0x400] 
                        : & ppu::nt_rdc_[0];
    ppu::bg_nametable = (value & 0x08) != 0 
                      ? & ppu::nt_rdc_[0x400] 
                        : & ppu::nt_rdc_[0];
    ppu::data_slot = (value & 0x10) >> 4;
    ppu::data_slot^= 1;

    ppu::graphics_mask &= ~15;

    if ( (value & 1) != 0 )
    {
      ppu::graphics_mask |= 8;
    }

    if ( (value & 32) != 0 )
    {
      ppu::graphics_mask |= 4;
    }

    if ( (value & 2) != 0 )
    {
      ppu::graphics_mask |= 2;
    }

    if ( (value & 4) != 0 )
    {
      ppu::graphics_mask |= 1;
    }
    ppu::graphics_render = ppu::graphics_renders[ppu::graphics_mask];

    ppu::control = value;
  }

  IR_FORCEINLINE void 
  vbk_write (uint8_t value) 
  { // FF4F - VBK - CGB Mode Only - VRAM Bank (R/W)
    // This register can be written to to change VRAM banks. Only bit 0 matters, all other bits are ignored.
    // 
    // VRAM bank 1
    // VRAM bank 1 is split like VRAM bank 0 ; 8000-97FF also stores tiles (just like in bank 0), 
    // which can be accessed the same way as (and at the same time as) bank 0 tiles. 
    // 9800-9FFF contains the attributes for the corresponding Tile Maps.
    // 
    // Reading from this register will return the number of the currently loaded VRAM bank in bit 0, and all other bits will be set to 1.
    T *gb = static_cast<T *> (this);

    if ( gb->gb::get_type () != 0 )
    {  
      uint32_t id = value & 1;

      gb->mpu::mmu[8].id = id;
      gb->mpu::mmu[9].id = id;
      gb->mpu::mmu[8].ptr= & ppu::vram[MEM_8K * id];
      gb->mpu::mmu[9].ptr= gb->mpu::mmu[8].ptr + MEM_4K;
    }
    ppu::vbk = value;
  }

  IR_FORCEINLINE
  void cgb_nametable_update ( nt_rdc &nt, nbyte *tile )
  {
    uint32_t tid = tile[0].memory;
    uint32_t attributes = tile[MEM_8K].memory;
  
    tile_map *p = & ppu::tile_map_[0];
              p = & p[768 * ( attributes & 8 )]; // bank, block offset
              p = & p[attributes & 7]; // palette block 4 offset
              p = & p[tid << 4]; // tile id offset 

    nt.entry[0] = & p[0];
    nt.entry[1] = & p[256 * 16 - ( tid & 0x80 ) * 32];
    nt.attributes = attributes;
    nt.id = tid;
    nt.filp_y_7l5x = 0;
    nt.filp_x = (attributes & 32);

    if ( (attributes & 64) != 0 )
    {
      nt.filp_y_7l5x = 7 << 5;
    }
    nt.pri = 
       (attributes & 0x80) != 0 
       ? 0x01000000 : 0x00000000;

  #ifdef _DEBUG
    if ( tid <= 0x7f )
    {
      if ( nt.entry[0]->mid[0] != tid 
       || nt.entry[0]->mid[1] != UINT32_MAX
        || nt.entry[1]->mid[1] != tid 
       || nt.entry[1]->mid[0] != UINT32_MAX )
      {
        IASSERT (0);
      }
    }
    else
    {
      if ( nt.entry[0]->mid[0] != tid 
       || nt.entry[0]->mid[1] != tid
        || nt.entry[1]->mid[0] != tid 
       || nt.entry[1]->mid[1] != tid )
      {
        IASSERT (0);
      }
    }
  #endif 
  }

  IR_FORCEINLINE
  void dmg_nametable_update ( nt_rdc &nt, uint32_t tid )
  {
    tile_map *p = & tile_map_[0];
              p = & p[tid << 2]; // tile id offset

    nt.entry[0] = & p[0];
    nt.entry[1] = & p[256 * 4 - ( tid & 0x80 ) * 8];
    nt.entry[2] = nt.entry[0] + 1;
    nt.entry[3] = nt.entry[1] + 1;

    nt.attributes = 0;
    nt.id = tid;

  #ifdef _DEBUG
    if ( tid <= 0x7f )
    {
      if ( nt.entry[0]->mid[0] != tid 
       || nt.entry[0]->mid[1] != UINT32_MAX
        || nt.entry[1]->mid[1] != tid 
       || nt.entry[1]->mid[0] != UINT32_MAX )
      {
        IASSERT (0);
      }
    }
    else
    {
      if ( nt.entry[0]->mid[0] != tid 
       || nt.entry[0]->mid[1] != tid
        || nt.entry[1]->mid[0] != tid 
       || nt.entry[1]->mid[1] != tid )
      {
        IASSERT (0);
      }
    }
  #endif 
  }

  IR_FORCEINLINE void 
  cgb_palette_access_write (uintptr_t slot, uint8_t value) 
  { // This register is used to address a byte in the CGBs Background Palette Memory. 
    //  Each two byte in that memory define a color value. The first 8 bytes define Color 0-3 of Palette 0 (BGP0), and so on for BGP1-7.
    // 
    //  Bit 0-5   Index (00-3F)
    //  Bit 7     Auto Increment  (0=Disabled, 1=Increment after Writing)
    // Data can be read/written to/from the specified index address through Register FF69. 
    // When the Auto Increment bit is set then the index is automatically incremented after each <write> to FF69. 
    // Auto Increment has no effect when <reading> from FF69, so the index must be manually incremented in that case.
    // Writing to FF69 during rendering still causes auto-increment to occur.
    // 
    // Unlike the following, this register can be accessed outside V-Blank and H-Blank.
    T *gb = static_cast<T *> (this);

    if ( gb->gb::get_type () != 0 )
    {  
      bcp_ocp &pal = ppu::bcp_ocp_[slot];

      pal.access = value;
      pal.increment = value >> 7;
      pal.source = value;

     // printf (" !!! slot:%d addr:%02x inc:%d       !!! \n", slot, value & 0x3f, pal.increment);
    }
  }

  IR_FORCEINLINE void 
  cgb_palette_data_write (uintptr_t slot, uint8_t value) 
  { 
    T *gb = static_cast<T *> (this);

    if ( gb->gb::get_type () != 0 )
    {  
      bcp_ocp &pal = ppu::bcp_ocp_[slot];
      pal_tmr *rec;
      tile_pixel &td = ppu::tile_data_;

      uint32_t t;
      uint32_t e;
      uint32_t cgb_r;
      uint32_t cgb_g;
      uint32_t cgb_b;
      uint32_t cgb_pix;

      t = pal.access;
          pal.access += pal.increment;
      t&= 0x3F;

      // printf (" === slot:%d addr:%02x col:%02x inc:%d === \n", slot, t & 0x3f, value, pal.increment );


      e = t & 0x3e;
          pal.pal[t] = value;
          rec = & pal.rec[t >> 3];
      t&= 7;

      if ( t >= 2 )
      { // not a transparent color, discard all associated pixel caches
  #ifdef _DEBUG
        uint32_t elements = rec->elements;
        while ( rec->elements > 0 )
        {
          tile_map *tm = rec->record[--rec->elements];
          IASSERT (tm != nullptr);
          IASSERT (tm->pix != nullptr);
          IASSERT (tm->epos < elements);

          td.rec[td.recs++] = tm->pix;

          tm->pix = nullptr;
        }
        IASSERT (rec->elements == 0);
  #else 
        IASSERT (0);
  #endif 
      }
      t >>= 1;

      // cgb color - little-endian RGB555
      // d4-d0:red 
      // d9-d5:green 
      // d14-d10:blue 
      cgb_pix = pal.pal[e] | static_cast<uint16_t>( pal.pal[e + 1] ) << 8;

      cgb_r = cgb_pix >> 0 & 0x1F;
      cgb_g = cgb_pix >> 5 & 0x1F;
      cgb_b = cgb_pix >>10 & 0x1F;
      cgb_r<<= 3;
      cgb_g<<= 3;
      cgb_b<<= 3;

      // printf (" color :%06x\n ", cgb_pix );

      * reinterpret_cast<uint32_t *> (& rec->session[t << 2]) = 
        ( cgb_r << 16 | cgb_g << 8 | cgb_b << 0 ) 
      | ((t != 0) ? pal.set_mask : pal.reset_mask);
    }
  }

  IR_FORCEINLINE void 
  dmg_palette_write ( uint32_t slot, uint8_t value, uint32_t force_mask = 0) 
  { 
    T *gb = static_cast<T *> (this);

    if ( gb->gb::get_type () == 0 )
    {
      if ( ppu::bgp_obp[slot] != value || force_mask != 0 )
      {
        pal_tmr &rec = ppu::pal_tmr_[slot];
        tile_pixel &td = ppu::tile_data_;

        uint32_t *col = reinterpret_cast<uint32_t *> (& ppu::gs_pal[slot << 4]);
        uint32_t *dst = reinterpret_cast<uint32_t *> (& rec.session[0]);

        dst[0] = col[value >> 0 & 0x03] & 0x00ffffff;
        dst[1] = col[value >> 2 & 0x03] & 0x00ffffff;
        dst[2] = col[value >> 4 & 0x03] & 0x00ffffff;
        dst[3] = col[value >> 6 & 0x03] & 0x00ffffff;

        dst[0] |= 0x00000000;
        dst[1] |= 0x80000000;
        dst[2] |= 0x80000000;
        dst[3] |= 0x80000000;

  #ifdef _DEBUG
        uint32_t elements = rec.elements;
        while ( rec.elements > 0 )
        {
          tile_map *tm = rec.record[--rec.elements];
          IASSERT (tm != nullptr);
          IASSERT (tm->pix != nullptr);
          IASSERT (tm->epos < elements);

          td.rec[td.recs++] = tm->pix;

          tm->pix = nullptr;
        }
        IASSERT (rec.elements == 0);
  #else 
        uint32_t blk4s = rec.elements >> 2;
        uint32_t blk1s = rec.elements & 3;

        tile_map **tm = & rec.record[0];
        uint8_t **rs = & td.rec[td.recs];

        td.recs += rec.elements;

        for ( uint32_t id = blk4s; id != 0; id-- )
        {
          tile_map *tm0 = tm[0];
          tile_map *tm1 = tm[1];
          tile_map *tm2 = tm[2];
          tile_map *tm3 = tm[3];

          rs[0] = tm0->pix;
          rs[1] = tm1->pix;
          rs[2] = tm2->pix;
          rs[3] = tm3->pix;

          tm0->pix = nullptr;
          tm1->pix = nullptr;
          tm2->pix = nullptr;
          tm3->pix = nullptr;

          tm += 4;
          rs += 4;
        }

        for ( uint32_t id = 0; id != blk1s; id++ )
        {
          tile_map *tm0 = tm[id];

          rs[id] = tm0->pix;

          tm0->pix = nullptr;
        }
        rec.elements = 0;
  #endif 
      }
    }
    ppu::bgp_obp[slot] = value;
  }

  template <size_t t_nMachineMode> IR_FORCEINLINE
  void sprite_update (uint32_t address, uint32_t value)
  {
    tile_map *tm;

    sprite &sp = ppu::sprite_[(address & 0xff) >> 2];
  
    uint32_t sub = address & 3;
    uint8_t *ps= & (reinterpret_cast<uint8_t *> (& sp)[sub]);
    uint32_t tid;

    ps[sub] = value;

    if ( sub <= 1 )
    { // position update
      sp.y2nx = sp.y - 16;

      if ( sp.x == 0 || sp.x >= 168 )
      { // calc combine cache
        sp.y2nx = 240;
      }
      sp.y2nx = 0 - sp.y2nx;
    }
    else
    { // tile id/attribute update
      IASSERT (sub <= 3);

      tid = sp.id;
      tid&= 0xfe;

      if ( t_nMachineMode == 0 )
      { // dmg machine mode 
        tm = & ppu::tile_map_[0];
        tm = & tm[sp.attributes >> 4 & 1]; // OBP0/OBP1 block offset
        tm = & tm[tid << 2]; // tile id offset 
        tm = & tm[2]; // sprite/nt_rdc block offset 
      }
      else
      { // cgb machine mode 
        IASSERT (t_nMachineMode == 1);

        tm = & ppu::tile_map_[0];
        tm = & tm[768 * ( sp.attributes & 8 )]; // bank, block offset 
        tm = & tm[sp.attributes & 7]; // palette block 4 offset 
        tm = & tm[tid << 4]; // tile id offset 
        tm = & tm[8]; // sprite offset 
      }

      if ( sub == 3 )
      { // attribute update 
        sp.filp_x = sp.attributes & 32;
        sp.filp_y_15l5x = 0;
        sp.filp_y_7l5x = 0;

        if ( (sp.attributes & 64) != 0 )
        {
          sp.filp_y_15l5x = 15 << 5;
          sp.filp_y_7l5x = 7 << 5;
        }
      } 
      sp.entry = tm;
      sp.odd_mask = 0;

      if ( (sp.id & 1) != 0 )
      {
        sp.odd_mask = 
          t_nMachineMode == 0 ? 4 : 16;
      }
  #ifdef _DEBUG
      tm = & tm[sp.odd_mask];

      if ( tm->mid[0] != sp.id )
      {
        IASSERT (0);
      }
  #endif
    }
  }

  IR_FORCEINLINE
  void compare_line (void)
  {
    T *gb = static_cast<T *> (this);

    ppu::ly_equal_mask = 0;

    if ( ppu::line == ppu::ly_cmp )
    {
      ppu::ly_equal_mask = UINT32_MAX;

      gb->gb::interrupt_requset (INT_STAT, ppu::lyc_sel_mask); 
    }
  }

  IR_FORCEINLINE
  void clk_ (int32_t clk)
  {
    if ( (ppu::clk = ppu::clk - clk) <= 0 )
    {
      clk_internal ();
    }
  } 

  IR_FORCEINLINE
  void oam_dma_ (uint32_t value)
  {
    T *gb = static_cast<T *> (this);

    ppu &ppu = * gb;
    T::mpu &mpu = * gb;

    uint32_t address = value << 8;
    uint32_t id = 20;

    sprite *sp = & ppu::sprite_[0];

    tile_map *tm;
    tile_map *tm2;

    nbyte *src = & mpu.mmu[address >> 12].ptr[address & 0xfff];

    uint32_t tid;
    uint32_t tid2;
  #ifdef _DEBUG
    uint32_t tid_;
    uint32_t tid2_;
  #endif 
    uint32_t attr;
    uint32_t attr2;

    if ( gb->gb::get_type () == 0 )
    { // dmg machine mode oam 
      do
      {     
        sp[0].y = src[0].memory;
        sp[0].x = src[1].memory;     
        sp[1].y = src[4].memory;
        sp[1].x = src[5].memory;
        sp[0].id = src[2].memory;
        sp[0].attributes = src[3].memory;
        sp[1].id = src[6].memory;
        sp[1].attributes = src[7].memory;

        tid = src[2].memory;
        attr= src[3].memory;
        tid2 = src[6].memory;
        attr2= src[7].memory;

        sp[0].odd_mask = (tid & 1) << 2;
        sp[1].odd_mask = (tid2& 1) << 2;
    #ifdef _DEBUG
        tid_ = tid;
        tid2_= tid2;
    #endif 
        tid &= 0xfe;
        tid2&= 0xfe;

        tm = & ppu::tile_map_[0];
        tm2= & ppu::tile_map_[0];
        tm = & tm[sp[0].attributes >> 4 & 1]; // OBP0/OBP1 block offset
        tm2= & tm2[sp[1].attributes >> 4 & 1]; // OBP0/OBP1 block offset
        tm = & tm[tid << 2]; // tile id offset 
        tm2= & tm2[tid2 << 2]; // tile id offset
        tm = & tm[2]; // sprite/nt_rdc block offset 
        tm2= & tm2[2]; // sprite/nt_rdc block offset 

        sp[0].entry = tm;
        sp[1].entry = tm2;

        sp[0].pri = attr & 0x80;
        sp[1].pri = attr2& 0x80;

    #ifdef _DEBUG
        tm = & tm[sp[0].odd_mask];
        tm2= & tm2[sp[1].odd_mask];

        if ( tm->mid[0] != tid_
          || tm2->mid[0] != tid2_)
        {
          IASSERT (0);
        }
    #endif 
        sp[0].filp_x = attr & 32;
        sp[0].filp_y_15l5x = 0;
        sp[0].filp_y_7l5x = 0;
        sp[0].y2nx = sp[0].y - 16;

        sp[1].filp_x = attr2 & 32;
        sp[1].filp_y_15l5x = 0;
        sp[1].filp_y_7l5x = 0;
        sp[1].y2nx = sp[1].y - 16;

        if ( (attr & 64) != 0 )
        {
          sp[0].filp_y_15l5x = 15 << 5;
          sp[0].filp_y_7l5x = 7 << 5;
        }

        if ( (attr2 & 64) != 0 )
        {
          sp[1].filp_y_15l5x = 15 << 5;
          sp[1].filp_y_7l5x = 7 << 5;
        }

        if ( sp[0].x == 0 || sp[0].x >= 168 )
        {
          sp[0].y2nx = 240;
        }

        if ( sp[1].x == 0 || sp[1].x >= 168 )
        {
          sp[1].y2nx = 240;
        }
        sp[0].y2nx = 0 - sp[0].y2nx;
        sp[1].y2nx = 0 - sp[1].y2nx;

        src += 8;

        sp += 2;
      } while (--id != 0);
    }
    else
    { // cgb machine mode 
      do
      {     
        sp[0].y = src[0].memory;
        sp[0].x = src[1].memory;     
        sp[1].y = src[4].memory;
        sp[1].x = src[5].memory;
        sp[0].id = src[2].memory;
        sp[0].attributes = src[3].memory;
        sp[1].id = src[6].memory;
        sp[1].attributes = src[7].memory;

        tid = src[2].memory;
        attr= src[3].memory;
        tid2 = src[6].memory;
        attr2= src[7].memory;

        sp[0].odd_mask = (tid & 1) << 4;
        sp[1].odd_mask = (tid2& 1) << 4;
    #ifdef _DEBUG
        tid_ = tid;
        tid2_= tid2;
    #endif 
        tid &= 0xfe;
        tid2&= 0xfe;

        tm = & ppu::tile_map_[0];
        tm2= & ppu::tile_map_[0];
        tm = & tm[768 * ( sp[0].attributes & 8 )]; // bank, block offset 
        tm2= & tm2[768 * ( sp[1].attributes & 8 )]; // bank, block offset 
        tm = & tm[sp[0].attributes & 7]; // palette block 4 offset 
        tm2= & tm2[sp[1].attributes & 7]; // palette block 4 offset 
        tm = & tm[tid << 4]; // tile id offset 
        tm2= & tm2[tid2 << 4]; // tile id offset 
        tm = & tm[8]; // sprite offset 
        tm2= & tm2[8]; // sprite offset

        sp[0].entry = tm;
        sp[1].entry = tm2;

        sp[0].pri = attr & 0x80;
        sp[1].pri = attr2& 0x80;

    #ifdef _DEBUG
        tm = & tm[sp[0].odd_mask];
        tm2= & tm2[sp[1].odd_mask];

        if ( tm->mid[0] != tid_
          || tm2->mid[0] != tid2_)
        {
          IASSERT (0);
        }
    #endif 
        sp[0].filp_x = attr & 32;
        sp[1].filp_x = attr2 & 32;
        sp[0].filp_y_15l5x = 0;
        sp[1].filp_y_15l5x = 0;
        sp[0].filp_y_7l5x = 0;
        sp[1].filp_y_7l5x = 0;
        sp[0].y2nx = sp[0].y - 16;
        sp[1].y2nx = sp[1].y - 16;

        if ( (attr & 64) != 0 )
        {
          sp[0].filp_y_15l5x = 15 << 5;
          sp[0].filp_y_7l5x = 7 << 5;
        }

        if ( (attr2 & 64) != 0 )
        {
          sp[1].filp_y_15l5x = 15 << 5;
          sp[1].filp_y_7l5x = 7 << 5;
        }

        if ( sp[0].x == 0 || sp[0].x >= 168 )
        {
          sp[0].y2nx = 240;
        }

        if ( sp[1].x == 0 || sp[1].x >= 168 )
        {
          sp[1].y2nx = 240;
        }
        sp[0].y2nx = 0 - sp[0].y2nx;
        sp[1].y2nx = 0 - sp[1].y2nx;

        src += 8;

        sp += 2;
      } while (--id != 0);
    }
  }

  IR_FORCEINLINE
  void hbl_dma (void)
  {
    T *gb = static_cast<T *> (this);

    ppu &ppu = * gb;
    T::mpu &mpu = * gb;

    uint16_t src = dma_.src_lo | dma_.src_hi << 8;
    uint16_t dst = dma_.dst_lo | dma_.dst_hi << 8;
    uint16_t src_= src;
    uint16_t dst_= dst;
    uint32_t lens= dma_.control;
    uint32_t id;

    nbyte *sptr;
    nbyte *dptr;

    src&= 0xFFF0;
    dst&= 0x1FF0;
    dst|= 0x8000;

    mpu.spin_ticks += 32; // 8 M cycles per 16 bytes

    sptr = & mpu.mmu[src >> 12].ptr[src & 0xfff];
    dptr = & mpu.mmu[dst >> 12].ptr[dst & 0xfff];

    dptr[0].memory = sptr[0].memory;
    dptr[1].memory = sptr[1].memory;
    dptr[2].memory = sptr[2].memory;
    dptr[3].memory = sptr[3].memory;
    dptr[4].memory = sptr[4].memory;
    dptr[5].memory = sptr[5].memory;
    dptr[6].memory = sptr[6].memory;
    dptr[7].memory = sptr[7].memory;
    dptr[8].memory = sptr[8].memory;
    dptr[9].memory = sptr[9].memory;
    dptr[10].memory = sptr[10].memory;
    dptr[11].memory = sptr[11].memory;
    dptr[12].memory = sptr[12].memory;
    dptr[13].memory = sptr[13].memory;
    dptr[14].memory = sptr[14].memory;
    dptr[15].memory = sptr[15].memory;

    if ( dst < 0x9800 )
    { // character data
      cgb_character_discard ( ppu::vbk & 1, dst );
    }
    else
    { // tile-id or attr table
      nt_rdc *p = & ppu::nt_rdc_[dst & 0x7FF];

      id = 4;

      do
      {
        cgb_nametable_update (p[0], & dptr[0]);
        cgb_nametable_update (p[1], & dptr[1]);
        cgb_nametable_update (p[2], & dptr[2]);
        cgb_nametable_update (p[3], & dptr[3]);

        p++;
        p++;
        p++;
        p++;

        id--;

        dptr += 4;
      } while (id != 0);
    }
    dst_ += 16;
    src_ += 16;

    dma_.src_lo = src_;
    dma_.src_hi = src_ >> 8;
    dma_.dst_lo = dst_;
    dma_.dst_hi = dst_ >> 8;

    if ( --dma_.control == 0xffffffff )
    { // close DMA
      gb->state &= ~0x8000;
    }
  }

  IR_FORCEINLINE
  void gp_dma (void)
  { 
    T *gb = static_cast <T *> (this);

    ppu<T> &ppu = * gb;
    mpu<T> &mpu = * gb;

    uint16_t src = dma_.src_lo | dma_.src_hi << 8;
    uint16_t dst = dma_.dst_lo | dma_.dst_hi << 8;
    uint16_t src_= src;
    uint16_t dst_= dst;
    uint32_t lens= dma_.control;
    uint32_t id;

    nbyte *sptr;
    nbyte *dptr;

    src&= 0xFFF0;
    dst&= 0x1FF0;
    dst|= 0x8000;

    mpu.spin_ticks += lens << 5; // 8 M cycles per 16 bytes

    sptr = & mpu.mmu[src >> 12].ptr[src & 0xfff];
    dptr = & mpu.mmu[dst >> 12].ptr[dst & 0xfff];

    do
    {
      dptr[0].memory = sptr[0].memory;
      dptr[1].memory = sptr[1].memory;
      dptr[2].memory = sptr[2].memory;
      dptr[3].memory = sptr[3].memory;
      dptr[4].memory = sptr[4].memory;
      dptr[5].memory = sptr[5].memory;
      dptr[6].memory = sptr[6].memory;
      dptr[7].memory = sptr[7].memory;
      dptr[8].memory = sptr[8].memory;
      dptr[9].memory = sptr[9].memory;
      dptr[10].memory = sptr[10].memory;
      dptr[11].memory = sptr[11].memory;
      dptr[12].memory = sptr[12].memory;
      dptr[13].memory = sptr[13].memory;
      dptr[14].memory = sptr[14].memory;
      dptr[15].memory = sptr[15].memory;

      if ( dst < 0x9800 )
      { // character data
        cgb_character_discard (ppu::vbk & 1, dst );

        dptr+= 16;
      }
      else
      { // tile-id or attr table
        nt_rdc *p = & ppu::nt_rdc_[dst & 0x7FF];

        id = 4;

        do
        {
          cgb_nametable_update (p[0], & dptr[0]);
          cgb_nametable_update (p[1], & dptr[1]);
          cgb_nametable_update (p[2], & dptr[2]);
          cgb_nametable_update (p[3], & dptr[3]);

          p++;
          p++;
          p++;
          p++;

          dptr++;
          dptr++;
          dptr++;
          dptr++;

          id--;

        } while (id != 0);
      }
      sptr+= 16;
      dst += 16;

      lens--;
    } while (lens != 0);

    dma_.control = 0xffffffff; // close DMA
  }

  IR_FORCEINLINE 
  void tile_map_discard_debug (tile_map &tm )
  {
#ifdef _DEBUG
    for (uint32_t id = 0; id != tile_data_.recs; id++ )
    {
      void *p = tile_data_.rec[id];
      IASSERT (p != nullptr);
    }

    pal_tmr *pt = tm.pal;

    if ( pt->elements != 0 )
    {
      IASSERT (pt->elements != 0);         
      IASSERT (tm.epos < pt->elements);

      for (uint32_t id = 0; id != pt->elements; id++ )
      {
        tile_map *p = pt->record[id];
        IASSERT (p != nullptr);
        IASSERT (p->pix != nullptr);
        IASSERT (p->epos < pt->elements);
        IASSERT (p->epos == id);
      }
    }
#endif 
  }

  IR_FORCEINLINE 
  void tile_map_discard_debug2 (tile_map &tm )
  {
#ifdef _DEBUG
    for (uint32_t id = 0; id != tile_data_.recs; id++ )
    {
      void *p = tile_data_.rec[id];
      IASSERT (p != nullptr);
    }

    pal_tmr *pt = tm.pal;

    if ( pt->elements != 0 )
    {
      for (uint32_t id = 0; id != pt->elements; id++ )
      {
        tile_map *p = pt->record[id];
        IASSERT (p != nullptr);
        IASSERT (p->pix != nullptr);
        IASSERT (p->epos < pt->elements);
        IASSERT (p->epos == id);
      }
    }
#endif 
  }

  IR_FORCEINLINE 
  void tile_map_discard (tile_map &tm )
  {              
    if ( tm.pix != nullptr )
    {
      tile_pixel &td = ppu::tile_data_;

#ifdef _DEBUG
      tile_map_discard_debug (tm);    

      IASSERT (tm.pal->elements > 0);

      tile_map *swp = tm.pal->record[--tm.pal->elements];
      IASSERT (swp != nullptr);

      tm.entry[0] = swp; // swap and free
      swp->entry =   tm.entry;
      swp->epos = tm.epos;

      td.rec[td.recs++] = tm.pix;

      IASSERT (tm.entry[0] != nullptr);

      tm.pix = nullptr;  
      tm.epos = 65533;

      tile_map_discard_debug2 (tm);
#else 
      tile_map *swp = tm.pal->record[--tm.pal->elements];
      IASSERT (swp != nullptr);

      td.rec[td.recs++] = tm.pix;

      tm.entry[0] = swp; // swap and free
      tm.pix = nullptr;  

      swp->entry = tm.entry;
#endif 
    }
  }

  IR_FORCEINLINE
  void dmg_character_discard (uint32_t address )
  {
  #if 1
    tile_map  *tm = & ppu::tile_map_[((address & 0x1FFF) >> 4) << 2];
  #else 
    tile_map  *tm = & ppu::tile_map_[(address & 0x1ff0) >> 2];
  #endif 
    tile_map_discard (tm[0]);
    tile_map_discard (tm[1]);
    tile_map_discard (tm[2]);
    tile_map_discard (tm[3]);
  }

  IR_FORCEINLINE
  void cgb_character_discard ( uintptr_t bank, uintptr_t address )
  {
    tile_map  *tm = & ppu::tile_map_[384 * 16 * bank];
               tm = & tm[address & 0x1FF0];

    tile_map_discard (tm[0]);
    tile_map_discard (tm[1]);
    tile_map_discard (tm[2]);
    tile_map_discard (tm[3]);
    tile_map_discard (tm[4]);
    tile_map_discard (tm[5]);
    tile_map_discard (tm[6]);
    tile_map_discard (tm[7]);
    tile_map_discard (tm[8]);
    tile_map_discard (tm[9]);
    tile_map_discard (tm[10]);
    tile_map_discard (tm[11]);
    tile_map_discard (tm[12]);
    tile_map_discard (tm[13]);
    tile_map_discard (tm[14]);
    tile_map_discard (tm[15]);
  }

  IR_FORCEINLINE
  void render_tile (tile_map *map )
  {
    tile_pixel &td = ppu::tile_data_;
    pal_tmr &pal_ = * map->pal;

    uint32_t *pal = reinterpret_cast<uint32_t *> (& pal_.session[0]);
    uint32_t *pix;
    uint8_t *chr = reinterpret_cast<uint8_t *> (map->chr);

    if ( td.recs != 0 )
    {
      pix = reinterpret_cast<uint32_t *> ( td.rec[--td.recs] );
      IASSERT (pix != nullptr);
    }
    else
    {
      pix = reinterpret_cast<uint32_t *>  (td.ptr);
                                           td.ptr += 256;
      IASSERT (pix != nullptr);

      td.cnts++;
    }
#ifdef _DEBUG
    map->epos = pal_.elements;
#endif 
    map->entry = & pal_.record[pal_.elements++];
    map->entry[0] = map;

    map->pix = reinterpret_cast<uint8_t *> (pix);

    tile_map_discard_debug (* map);

    // render gameboy grayscale image 
    // 
    // each 16 bytes represents a group of 8*8 grayscale pixels, \
    // each pixel only contains two bits of information, \
    // so a combination can only have four colors at most.
    // 
    // 0x0000~0x000F 8*8 pixel chr data ...
    // 0x0010~0x001F 8*8 pixel chr data ...
    // 0x0020~0x002F ............
    // 8*8 pixel array is as follows...
    // 
    // eight pixels in the first row         
    //                             YPOS:    00000000
    //                             XPOS:    01234567
    //                              BIT:    76543210
    // 0x0001 (color high bit set)  0x7C -> 01010100 
    // 0x0000 (color low bit set)   0xE3 -> 11100011 
    //                                      ||||||||_ col:01 
    //                                      |||||||__ col:01 
    //                                      ||||||___ col:10 
    //                                      |||||____ col:00 
    //                                      ||||_____ col:10 
    //                                      |||______ col:01 
    //                                      ||_______ col:11 
    //                                      |________ col:01
    // eight pixels in the second row 
    //                             YPOS:    11111111
    //                             XPOS:    01234567
    //                              BIT:    76543210
    // 0x0003 (color high bit set)  0x94 -> 10010100 
    // 0x0002 (color low bit set)   0xBA -> 10111100 
    //                                      ||||||||_ col:00 
    //                                      |||||||__ col:00 
    //                                      ||||||___ col:11 
    //                                      |||||____ col:01 
    //                                      ||||_____ col:11 
    //                                      |||______ col:01 
    //                                      ||_______ col:00 
    //                                      |________ col:11
    // and so on ......
  #ifdef IR_FAST_X86_SIMD_VECTOR
    // HI-128 XX 7H XX 7L | XX 6H XX 6L | XX 5H XX 5L | XX 4H XX 4L                              
    // LO-128 XX 3H XX 3L | XX 2H XX 2L | XX 1H XX 1L | XX 0H XX 0L
    __m256i src = _mm256_shuffle_epi8 (   _mm256_loadu_si256 ( reinterpret_cast<const __m256i *> (chr) )
                                    , _mm256_set_epi32 ( 0x0E0A0602, 0x0E0A0602, 0x0C080400, 0x0C080400
                                                    , 0x0E0A0602, 0x0E0A0602, 0x0C080400, 0x0C080400 )  );
    __m256i *dst = reinterpret_cast<__m256i *>(& pix[0]);
    // HI-128 7H 6H 5H 4H | 7H 6H 5H 4H | 7L 6L 5L 4L | 7L 6L 5L 4L
    // LO-128 3H 2H 1H 0H | 3H 2H 1H 0H | 3L 2L 1L 0L | 3L 2L 1L 0L
    __m256i msk = _mm256_set_epi32 ( 0x01010101, 0x02020202, 0x04040404, 0x08080808
                                  , 0x10101010, 0x20202020, 0x40404040, 0x80808080 );
    __m256i sft = _mm256_set_epi32 ( 0x00000000, 0x00000001, 0x00000002, 0x00000003
                                  , 0x00000004, 0x00000005, 0x00000006, 0x00000007 );
    __m256i d = _mm256_loadu_si256 (reinterpret_cast<const __m256i *>(pal));
    __m256i p;
    __m256i b;
    __m256i q;

    b = _mm256_and_si256  ( _mm256_permute4x64_epi64 ( src, 0x00 ), msk );
    q = _mm256_and_si256  ( _mm256_permute4x64_epi64 ( src, 0x55 ), msk );
    b = _mm256_srlv_epi32 ( b, sft );
    q = _mm256_srlv_epi32 ( q, sft );
    p = _mm256_or_si256 ( b, _mm256_slli_epi32 ( q, 1 ) );
    
    _mm256_storeu_si256 ( & dst[0], _mm256_permutevar8x32_epi32 (d, p));
    _mm256_storeu_si256 ( & dst[1], _mm256_permutevar8x32_epi32 (d, _mm256_srli_epi32 ( p, 8 )));
    _mm256_storeu_si256 ( & dst[2], _mm256_permutevar8x32_epi32 (d, _mm256_srli_epi32 ( p, 16 )));
    _mm256_storeu_si256 ( & dst[3], _mm256_permutevar8x32_epi32 (d, _mm256_srli_epi32 ( p, 24 )));   
    
    b = _mm256_and_si256  ( _mm256_permute4x64_epi64 ( src, 0xaa ), msk );
    q = _mm256_and_si256  ( _mm256_permute4x64_epi64 ( src, 0xff ), msk );
    b = _mm256_srlv_epi32 ( b, sft );
    q = _mm256_srlv_epi32 ( q, sft );
    p = _mm256_or_si256 ( b, _mm256_slli_epi32 ( q, 1 ) );

    _mm256_storeu_si256 ( & dst[4], _mm256_permutevar8x32_epi32 (d, p));
    _mm256_storeu_si256 ( & dst[5], _mm256_permutevar8x32_epi32 (d, _mm256_srli_epi32 ( p, 8 )));
    _mm256_storeu_si256 ( & dst[6], _mm256_permutevar8x32_epi32 (d, _mm256_srli_epi32 ( p, 16 )));
    _mm256_storeu_si256 ( & dst[7], _mm256_permutevar8x32_epi32 (d, _mm256_srli_epi32 ( p, 24 )));     
  #else 
    for ( id = 0; id != 8; id++ )
    {
      pix[0] = pal[(chr[0] >> 7) & 1 | (chr[2] >> 6) & 2];
      pix[1] = pal[(chr[0] >> 6) & 1 | (chr[2] >> 5) & 2];
      pix[2] = pal[(chr[0] >> 5) & 1 | (chr[2] >> 4) & 2];
      pix[3] = pal[(chr[0] >> 4) & 1 | (chr[2] >> 3) & 2];
      pix[4] = pal[(chr[0] >> 3) & 1 | (chr[2] >> 2) & 2];
      pix[5] = pal[(chr[0] >> 2) & 1 | (chr[2] >> 1) & 2];
      pix[6] = pal[(chr[0] >> 1) & 1 | (chr[2] >> 0) & 2];
      pix[7] = pal[(chr[0] >> 0) & 1 | (chr[2] << 1) & 2];

      pix2 += 8;
      chr += 2;
    }
  #endif 
  }

  void reset (void)
  {
    T *gb = static_cast<T *> (this);

    nt_rdc *nt = & nt_rdc_[0];

    nbyte *ntp = & ppu::vram[0x1C00];
    nbyte *chr = & ppu::vram[0];

    tile_map *tm = & ppu::tile_map_[0];
    tile_map *tm_= tm;

    uintptr_t x;
 // uintptr_t y;
    uintptr_t id;

    ppu::win_counter = 0;

    ppu::tile_data_.ptr = & ppu::tile_data_.data[0];
    ppu::tile_data_.ptr+=31;
    ppu::tile_data_.ptr = reinterpret_cast<uint8_t *> ( reinterpret_cast<uintptr_t> ( ppu::tile_data_.ptr ) & -32 );
    ppu::tile_data_.cnts = 0;

    ppu::tile_data_.recs= 0;

    ppu::mode = 2;
    ppu::line = 0;
    ppu::clk = 80;

    ppu::win_nametable = & ppu::nt_rdc_[0];
    ppu::bg_nametable = & ppu::nt_rdc_[0];

    ppu::data_slot = 0;

    ppu::control = 0x91;
    ppu::scr_x = 
    ppu::scr_y = 
    ppu::win_x =
    ppu::win_y =
    ppu::line = 0;
    ppu::vbk = 0xfe;
    ppu::oam_dma = 0xff;

    ppu::dma_.src_lo = 0xff;
    ppu::dma_.src_hi = 0xff;
    ppu::dma_.dst_lo = 0xff;
    ppu::dma_.dst_hi = 0xff;
    ppu::dma_.control = 0xffffffff;
    
    ppu::graphics_mask = 0;

    if ( gb->gb::get_type () == 0 )
    { // dmg machine mode
      memset ( & ppu::vram[0], 0, MEM_8K );

      for ( id = 0; id != 1024; id++ )
      { // build nametable
        nt[0x000].entry[0] = 
        nt[0x000].entry[1] = 

        nt[0x400].entry[0] = 
        nt[0x400].entry[1] = & ppu::tile_map_[0];

        nt[0x000].entry[2] = 
        nt[0x000].entry[3] = 

        nt[0x400].entry[2] = 
        nt[0x400].entry[3] = & ppu::tile_map_[2];

        nt[0x000].pri = 0x01000000;
        nt[0x400].pri = 0x01000000;

    #ifdef _DEBUG
        nt[0x000].x = 
        nt[0x400].x = id & 31;

        nt[0x000].y = 
        nt[0x400].y = id >> 5;

        nt[0x000].pid = 0;
        nt[0x400].pid = 1;
    #endif 
        nt++;
      }

      for ( id = 0; id != 40; id++ )
      { // build sprite settings
        ppu::sprite_[id].entry = tm;
      } 

      for ( id = 0; id != 384; id++ )
      { // build nt_rdc settings
        tm[0].chr = chr;
        tm[1].chr = chr;
        tm[2].chr = chr;
        tm[3].chr = chr;

        tm[0].pal = & ppu::pal_tmr_[0];
        tm[1].pal = & ppu::pal_tmr_[1];
        tm[2].pal = & ppu::pal_tmr_[2];
        tm[3].pal = & ppu::pal_tmr_[3];

        tm[0].pix = nullptr;
        tm[1].pix = nullptr;
        tm[2].pix = nullptr;
        tm[3].pix = nullptr;

        tm[0].entry = nullptr;
        tm[1].entry = nullptr;
        tm[2].entry = nullptr;
        tm[3].entry = nullptr;

  #ifdef _DEBUG
        if ( id <= 127 )
        {
          tm[0].mid[0] = 
          tm[1].mid[0] = 
          tm[2].mid[0] = 
          tm[3].mid[0] = id;
          tm[0].mid[1] = 
          tm[1].mid[1] = 
          tm[2].mid[1] = 
          tm[3].mid[1] = 0xffffffff;
        }
        else if ( id <= 255 )
        {
          tm[0].mid[0] = 
          tm[1].mid[0] = 
          tm[2].mid[0] = 
          tm[3].mid[0] = 
          tm[0].mid[1] = 
          tm[1].mid[1] = 
          tm[2].mid[1] = 
          tm[3].mid[1] = id;
        }
        else
        { 
          IASSERT (id < 384);

          tm[0].mid[0] = 
          tm[1].mid[0] = 
          tm[2].mid[0] = 
          tm[3].mid[0] = 0xffffffff;

          tm[0].mid[1] = 
          tm[1].mid[1] = 
          tm[2].mid[1] = 
          tm[3].mid[1] = id - 256;
        }
        tm[0].pid = 
        tm[1].pid = 
        tm[2].pid = 
        tm[3].pid = 0;

        tm[0].mid[2] = 
        tm[1].mid[2] = 
        tm[2].mid[2] = 
        tm[3].mid[2] = id;

        tm[0].sid = 0;
        tm[1].sid = 1;
        tm[2].sid = 2;
        tm[3].sid = 3;
  #endif 
        chr += 16;
        tm += 4;
      }
      ppu::pal_tmr_[0].elements = 0;
      ppu::pal_tmr_[1].elements = 0;
      ppu::pal_tmr_[2].elements = 0;
      ppu::pal_tmr_[3].elements = 0;

      dmg_palette_write ( 0, 0xfc );
    }
    else
    {
      memset ( & ppu::vram[0], 0, MEM_8K );
      memset ( & ppu::vram[MEM_8K], 0, MEM_8K );

      for ( id = 0; id != 1024; id++ )
      { // build nametable
        nt[0x000].entry[0] = 
        nt[0x000].entry[1] = 

        nt[0x400].entry[0] = 
        nt[0x400].entry[1] = & ppu::tile_map_[0];

        nt[0x000].entry[2] = 
        nt[0x000].entry[3] = 

        nt[0x400].entry[2] = 
        nt[0x400].entry[3] = & ppu::tile_map_[2];

        nt[0x000].pri = 0x01000000;
        nt[0x400].pri = 0x01000000;

    #ifdef _DEBUG
        nt[0x000].x = 
        nt[0x400].x = id & 31;

        nt[0x000].y = 
        nt[0x400].y = id >> 5;

        nt[0x000].pid = 0;
        nt[0x400].pid = 1;
    #endif 
        nt++;
      }

      for ( id = 0; id != 40; id++ )
      { // build sprite settings
        ppu::sprite_[id].entry = tm;
      } 

      for ( x = 0; x != 2; x++ )
      {
        chr = & vram[MEM_8K * x];

        for ( id = 0; id != 384; id++ )
        { // build nt_rdc settings
          tm[0].chr = chr;
          tm[1].chr = chr;
          tm[2].chr = chr;
          tm[3].chr = chr;
          tm[4].chr = chr;
          tm[5].chr = chr;
          tm[6].chr = chr;
          tm[7].chr = chr;
          tm[8].chr = chr;
          tm[9].chr = chr;
          tm[10].chr = chr;
          tm[11].chr = chr;
          tm[12].chr = chr;
          tm[13].chr = chr;
          tm[14].chr = chr;
          tm[15].chr = chr;

          tm[0].pal = & ppu::pal_tmr_[0];
          tm[1].pal = & ppu::pal_tmr_[1];
          tm[2].pal = & ppu::pal_tmr_[2];
          tm[3].pal = & ppu::pal_tmr_[3];
          tm[4].pal = & ppu::pal_tmr_[4];
          tm[5].pal = & ppu::pal_tmr_[5];
          tm[6].pal = & ppu::pal_tmr_[6];
          tm[7].pal = & ppu::pal_tmr_[7];
          tm[8].pal = & ppu::pal_tmr_[8];
          tm[9].pal = & ppu::pal_tmr_[9];
          tm[10].pal = & ppu::pal_tmr_[10];
          tm[11].pal = & ppu::pal_tmr_[11];
          tm[12].pal = & ppu::pal_tmr_[12];
          tm[13].pal = & ppu::pal_tmr_[13];
          tm[14].pal = & ppu::pal_tmr_[14];
          tm[15].pal = & ppu::pal_tmr_[15];

          tm[0].entry = nullptr;
          tm[1].entry = nullptr;
          tm[2].entry = nullptr;
          tm[3].entry = nullptr;
          tm[4].entry = nullptr;
          tm[5].entry = nullptr;
          tm[6].entry = nullptr;
          tm[7].entry = nullptr;
          tm[8].entry = nullptr;
          tm[9].entry = nullptr;
          tm[10].entry = nullptr;
          tm[11].entry = nullptr;
          tm[12].entry = nullptr;
          tm[13].entry = nullptr;
          tm[14].entry = nullptr;
          tm[15].entry = nullptr;

    #ifdef _DEBUG
          if ( id <= 127 )
          {
            tm[0].mid[0] = 
            tm[1].mid[0] = 
            tm[2].mid[0] = 
            tm[3].mid[0] = 
            tm[4].mid[0] = 
            tm[5].mid[0] = 
            tm[6].mid[0] = 
            tm[7].mid[0] = 
            tm[8].mid[0] = 
            tm[9].mid[0] = 
            tm[10].mid[0] = 
            tm[11].mid[0] = 
            tm[12].mid[0] = 
            tm[13].mid[0] = 
            tm[14].mid[0] = 
            tm[15].mid[0] = id;

            tm[0].mid[1] = 
            tm[1].mid[1] = 
            tm[2].mid[1] = 
            tm[3].mid[1] = 
            tm[4].mid[1] = 
            tm[5].mid[1] = 
            tm[6].mid[1] = 
            tm[7].mid[1] = 
            tm[8].mid[1] = 
            tm[9].mid[1] = 
            tm[10].mid[1] = 
            tm[11].mid[1] = 
            tm[12].mid[1] = 
            tm[13].mid[1] = 
            tm[14].mid[1] = 
            tm[15].mid[1] = 0xffffffff;
          }
          else if ( id <= 255 )
          {
            tm[0].mid[0] = 
            tm[1].mid[0] = 
            tm[2].mid[0] = 
            tm[3].mid[0] = 
            tm[4].mid[0] = 
            tm[5].mid[0] = 
            tm[6].mid[0] = 
            tm[7].mid[0] = 
            tm[8].mid[0] = 
            tm[9].mid[0] = 
            tm[10].mid[0] = 
            tm[11].mid[0] = 
            tm[12].mid[0] = 
            tm[13].mid[0] = 
            tm[14].mid[0] = 
            tm[15].mid[0] = 

            tm[0].mid[1] = 
            tm[1].mid[1] = 
            tm[2].mid[1] = 
            tm[3].mid[1] = 
            tm[4].mid[1] = 
            tm[5].mid[1] = 
            tm[6].mid[1] = 
            tm[7].mid[1] = 
            tm[8].mid[1] = 
            tm[9].mid[1] = 
            tm[10].mid[1] = 
            tm[11].mid[1] = 
            tm[12].mid[1] = 
            tm[13].mid[1] = 
            tm[14].mid[1] = 
            tm[15].mid[1] = id;
          }
          else
          { 
            IASSERT (id < 384);

            tm[0].mid[0] = 
            tm[1].mid[0] = 
            tm[2].mid[0] = 
            tm[3].mid[0] = 
            tm[4].mid[0] = 
            tm[5].mid[0] = 
            tm[6].mid[0] = 
            tm[7].mid[0] = 
            tm[8].mid[0] = 
            tm[9].mid[0] = 
            tm[10].mid[0] = 
            tm[11].mid[0] = 
            tm[12].mid[0] = 
            tm[13].mid[0] = 
            tm[14].mid[0] = 
            tm[15].mid[0] = 0xffffffff;

            tm[0].mid[1] = 
            tm[1].mid[1] = 
            tm[2].mid[1] = 
            tm[3].mid[1] = 
            tm[4].mid[1] = 
            tm[5].mid[1] = 
            tm[6].mid[1] = 
            tm[7].mid[1] = 
            tm[8].mid[1] = 
            tm[9].mid[1] = 
            tm[10].mid[1] = 
            tm[11].mid[1] = 
            tm[12].mid[1] = 
            tm[13].mid[1] = 
            tm[14].mid[1] = 
            tm[15].mid[1] = id - 256;
          }
          tm[0].pid = 
          tm[1].pid = 
          tm[2].pid = 
          tm[3].pid = 
          tm[4].pid = 
          tm[5].pid = 
          tm[6].pid = 
          tm[7].pid = 
          tm[8].pid = 
          tm[9].pid = 
          tm[10].pid = 
          tm[11].pid = 
          tm[12].pid = 
          tm[13].pid = 
          tm[14].pid = 
          tm[15].pid = x;

          tm[0].mid[2] = 
          tm[1].mid[2] = 
          tm[2].mid[2] = 
          tm[3].mid[2] = 
          tm[4].mid[2] = 
          tm[5].mid[2] = 
          tm[6].mid[2] = 
          tm[7].mid[2] = 
          tm[8].mid[2] = 
          tm[9].mid[2] = 
          tm[10].mid[2] = 
          tm[11].mid[2] = 
          tm[12].mid[2] = 
          tm[13].mid[2] = 
          tm[14].mid[2] = 
          tm[15].mid[2] = id;

          tm[0].sid = 0;
          tm[1].sid = 1;
          tm[2].sid = 2;
          tm[3].sid = 3;
          tm[4].sid = 4;
          tm[5].sid = 5;
          tm[6].sid = 6;
          tm[7].sid = 7;
          tm[8].sid = 8;
          tm[9].sid = 9;
          tm[10].sid = 10;
          tm[11].sid = 11;
          tm[12].sid = 12;
          tm[13].sid = 13;
          tm[14].sid = 14;
          tm[15].sid = 15;
    #endif 
          chr+= 16;
          tm += 16;
        }
      }
      graphics_mask |= 16;
    }
    stat_write ( 0x81 );

    ppu::graphics_render = ppu::graphics_renders[ppu::graphics_mask];
  }
};



template <class T>
class gb : public apu<T>
         , public joypad<T>
         , public timer<T>
         , public ppu<T>
         , public mpu<T>
         , public cart<T>
{
public:
   typedef ::apu<T> apu;
   typedef ::joypad<T> joypad;
   typedef ::timer<T> timer;
   typedef ::ppu<T> ppu;
   typedef ::mpu<T> mpu;
   typedef ::cart<T> cart;

   int32_t state; //  d31:block 
                  //  d30-d24:truncate sta* 
                  //        0:normal 
                  //        1:before render post edge interrupt
                  //        2:before vbl edge interrupt
                  //        3:before hbl edge interrupt 
                  //        4:before oam edge interrupt 
                  //        5:before draw edge interrupt 
                  //        6:before scanline ly_cmp edge interrupt
                  //        7:before scanline next edge interrupt
                  //        8:before timer edge interrupt
                  //  d15:h-blank dma work 
                  //  d14:cassette has been read
                  //  
                  //  d7-d0:emulate mode 0:dmg 1:cgb 2:sgb


  int resume (void)
  {
    if ( (state & 0x4000) != 0 )
    {
      state &= ~0x80000000;
    }    
    else
    {
      state |= 0x80000000;
    }
    return 0;
  }

  int set_audio_sample (uint32_t samples_)
  {
    IASSERT (samples_ != 0);

    apu::sample_reload =
    apu::sample_counter= 4194304 / samples_;
    return 0;
  }

  int32_t 
  frame (void)
  {
    int32_t op_ticks; // normal speed 
    int32_t op_ticks2x;
 
    uint16_t sbl;
    uint8_t t8;
    uint16_t t16;
    uint8_t in8;
    uint8_t out8;

    nbyte *ctr;

    if ( gb::state >= 0 )
    {
      if (mpu::spin_ticks != 0)
      {
        IASSERT (mpu::spin_ticks >= 0);

        op_ticks = mpu::spin_ticks;

        if ( mpu::spin_ticks > 16 )
        {
          op_ticks = 16;

          mpu::spin_ticks -= 16;
        }
        else
        {
          mpu::spin_ticks = 0;
        }
        goto L1;
      }
      sbl = mpu::pc & 0xfff;

      if ( mpu::pc <= 0xfdff )
      {
        ctr = & mpu::mmu[mpu::pc >> 12].ptr[sbl];
      }
      else
      {
        ctr = & mpu::io[sbl & 0x1ff];
      }

#define OPCODE1 ((sbl <= 0xffe) \
  ? ctr[1].memory : gb::read (mpu::pc + 1))
#define OPCODE2 ((sbl <= 0xffd) \
  ? ctr[2].memory : gb::read (mpu::pc + 2))

      if ( ((t8 = ctr->trace) & mpu::ss_exec_bp_rmask) != 0)
      { // exec breakpoint 
        if ( (t8 & 1) != 0 )
        { // exec breakpoint (fast) 
          gb::state |= 0x80000000;
          return 1;
        }
        else if ( (t8 & 2) != 0 )
        { // exec breakpoint (condition) 
          if ( mpu::bp_expr_rt ( mpu::pc) == 0 )
          {
            gb::state |= 0x80000000;
            return 1;
          }
        }
        else
        {
          uint32_t ext_mode = t8 >> 6;

          if ( ext_mode == 1 )
          { // exec breakpoint (step-over) 
            if ( mpu::invoke_collector.elements == mpu::temp_bp_confirm )
            {
              gb::state |= 0x80000000;
              return 1;
            }
          }
          else if ( ext_mode == 2 )
          { // current breakpoint (cursor) 
            gb::state |= 0x80000000;
            return 1;
          }
          else if ( ext_mode == 3 )
          { // current breakpoint (step-out) 
            if ( mpu::invoke_collector.elements == mpu::temp_bp_confirm )
            {
              gb::state |= 0x80000000;
              return 1;
            }
          }
        }
      }

#define READ_HOOK_(address, is16bit)                      \
  {                                                       \
    struct addr_map *bank = & mpu::mmu[(address) >> 12];     \
    uint8_t msk = bank->ptr[(address) & 0xfff].trace;     \
                                                          \
    if ( (msk & mpu::ss_rd_bp_rmask) != 0)                \
    {                                                     \
      if ( (msk & 0x04) != 0 )                            \
      { /* memory access read/write breakpoint (fast) */  \
        gb::state |= 0x80000000;                         \
        return 1;                                         \
      }                                                   \
      else                                                \
      { /* memory access read breakpoint (cond) */        \
        mpu::bp_type = !!is16bit;                         \
                                                          \
        mpu::m_read_addr = address;                       \
                                                          \
        if (bp_expr_rt (mpu::m_write_addr) == 0)     \
        {                                                 \
          return 1;                                       \
        }                                                 \
      }                                                   \
    }                                                     \
  }   

#define READ_HOOK(address)\
    READ_HOOK_(address, 0)
#define READ_HOOK2(address)\
    READ_HOOK_(address, 0x400)

#define WRITE_HOOK_(address, val, is16bit)                \
  {                                                       \
    mpu::addr_map &bank = mpu::mmu[(address) >> 12];      \
    uint8_t msk = bank.ptr[(address) & 0xfff].trace;     \
                                                          \
    if ( (msk & mpu::ss_wr_bp_rmask) != 0)                \
    {                                                     \
      if ( (msk & 0x10) != 0 )                            \
      { /* memory access read/write breakpoint (fast) */  \
        gb::state |= 0x80000000;                         \
        return 1;                                         \
      }                                                   \
      else                                                \
      { /* memory access write breakpoint (cond) */       \
        mpu::bp_type = 4 + !!is16bit;                     \
                                                          \
        mpu::m_write_addr = address;                      \
        mpu::m_write_val = val;                           \
                                                          \
        if (mpu::bp_expr_rt (mpu::m_write_addr) == 0)     \
        {                                                 \
          return 1;                                       \
        }                                                 \
      }                                                   \
    }                                                     \
  }   

#define WRITE_HOOK(address, val)\
    WRITE_HOOK_(address, val, 0)
#define WRITE_HOOK2(address, val)\
    WRITE_HOOK_(address, val, 1)

#define ACCESS_HOOK(address, val)                        \
  {                                                      \
    addr_map &bank = mpu::mmu[(address) >> 12];    \
    uint8_t msk = bank.ptr[(address) & 0xfff].trace;    \
                                                         \
    if ( (msk & mpu::ss_acc_bp_rmask) != 0)              \
    {                                                    \
      if ( (msk & 0x14) != 0 )                           \
      { /* memory access read/write breakpoint (fast) */ \
        gb::state |= 0x80000000;                        \
        return 1;                                        \
      }                                                  \
      else                                               \
      {                                                  \
        if ( (msk & 0x08) != 0 )                         \
        { /* memory access read breakpoint (cond) */     \
          mpu::bp_type = 0;                              \
                                                         \
          mpu::m_read_addr = address;                    \
                                                         \
          if (bp_expr_rt (mpu::m_write_addr) == 0)  \
          {                                              \
            return 1;                                    \
          }                                              \
        }                                                \
                                                         \
        if ( (msk & 0x20) != 0 )                         \
        { /* memory access write breakpoint (cond) */    \
          mpu::bp_type = 4;                              \
                                                         \
          mpu::m_write_addr = address;                   \
          mpu::m_write_val = val;                        \
                                                         \
          if (bp_expr_rt (mpu::m_write_addr) == 0)  \
          {                                              \
            return 1;                                    \
          }                                              \
        }                                                \
      }                                                  \
    }                                                    \
  }                                                               

#define COND_NC ((mpu::cf & 0x100) == 0)
#define COND_C ((mpu::cf & 0x100) != 0)
#define COND_NZ (mpu::zf != 0)
#define COND_Z (mpu::zf == 0)
#define COND_ALWAYS (1)
#define COND_NEVER (0)

#define OPCL(op_len, op_clk)  \
  mpu::pc += op_len;          \
                              \
  op_ticks = op_clk;

#define ATOM16_(op_start, reg) \
  case op_start:               \
    reg++;                     \
                               \
    OPCL (1, 8)                \
    break;                     \
                               \
  case op_start + 8:           \
    reg--;                     \
                               \
    OPCL (1, 8)                \
    break;

#define LD_D8_(opcode, reg) \
  case opcode:              \
    reg = OPCODE1;          \
                            \
    OPCL (2, 8);            \
    break;

#define ADD_()                                       \
  {                                                  \
    uint16_t src = in8;                              \
    uint16_t dst = mpu::a;                           \
    uint16_t res = dst + src;                        \
                                                     \
    mpu::zf = (uint8_t) res;                         \
    mpu::nf = 0;                                     \
    mpu::hf = (dst & 0xF) + (src & 0x0F);            \
    mpu::cf = res;                                   \
                                                     \
    mpu::a = (uint8_t) res;                          \
  }                                                  

#define ADD16_(opcode, src)                          \
  case opcode:                                       \
  {                                                  \
    uint32_t res = (uint32_t) mpu::hl + src;         \
                                                     \
    mpu::nf = 0;                                     \
    mpu::hf = (mpu::hl & 0x0FFF) + (src & 0x0FFF);   \
    mpu::hf >>= 8;                                   \
    mpu::cf = res;                                   \
    mpu::cf >>= 8;                                   \
                                                     \
    mpu::hl = res;                                   \
                                                     \
    OPCL (1, 8)                                      \
  }                                                  \
  break;                                              

#define ADD16_X_(opcode, dst, ticks)                  \
  case opcode:                                        \
  {                                                   \
    int16_t e = static_cast<int8_t> (OPCODE1);        \
    uint32_t res = static_cast<uint32_t>( mpu::sp)    \
                + static_cast<uint16_t> ( e );        \
                                                      \
    mpu::zf = UINT32_MAX;                             \
    mpu::nf = 0;                                      \
    mpu::hf = (mpu::sp & 0xF) + (e & 0x0F);           \
    mpu::cf = (mpu::sp & 0xFF) + (e & 0x0FF);         \
                                                      \
    dst = res;                                        \
                                                      \
    OPCL (2, ticks)                                   \
  }                                                   \
  break;           

#define ADC_()                                       \
  {                                                  \
    uint16_t carry = (mpu::cf & 0x100) >> 8;         \
    uint16_t src = in8;                              \
    uint16_t dst = mpu::a;                           \
    uint16_t res = dst + src + carry;                \
                                                     \
    mpu::zf = (uint8_t) res;                         \
    mpu::nf = 0;                                     \
    mpu::hf = (dst & 0xF) + (src & 0x0F) + carry;    \
    mpu::cf = res;                                   \
                                                     \
    mpu::a = (uint8_t) res;                          \
  }                                                  

#define SUB_()                                       \
  {                                                  \
    uint16_t src = in8;                              \
    uint16_t dst = mpu::a;                           \
    uint16_t res = dst - src;                        \
                                                     \
    mpu::zf = (uint8_t) res;                         \
    mpu::nf = UINT32_MAX;                            \
    mpu::hf = (dst & 0xF) - (src & 0x0F);            \
    mpu::cf = res;                                   \
                                                     \
    mpu::a = (uint8_t) res;                          \
  } 

#define SBC_()                                       \
  {                                                  \
    uint16_t carry = (mpu::cf & 0x100) >> 8;         \
    uint16_t src = in8;                              \
    uint16_t dst = mpu::a;                           \
    uint16_t res = dst - src - carry;                \
                                                     \
    mpu::zf = (uint8_t) res;                         \
    mpu::nf = UINT32_MAX;                            \
    mpu::hf = (dst & 0xF) - (src & 0x0F) - carry;    \
    mpu::cf = res;                                   \
                                                     \
    mpu::a = (uint8_t) res;                          \
  }       

#define AND_()            \
  {                       \
    mpu::a &= in8;        \
                          \
    mpu::zf = mpu::a;     \
    mpu::nf = 0;          \
    mpu::hf = UINT32_MAX; \
    mpu::cf = 0;          \
  }                       

#define XOR_()            \
  {                       \
    mpu::a ^= in8;        \
                          \
    mpu::zf = mpu::a;     \
    mpu::nf = 0;          \
    mpu::hf = 0;          \
    mpu::cf = 0;          \
  }                       

#define ORA_()            \
  {                       \
    mpu::a |= in8;        \
                          \
    mpu::zf = mpu::a;     \
    mpu::nf = 0;          \
    mpu::hf = 0;          \
    mpu::cf = 0;          \
  }   

#define CMP_()                              \
  {                                         \
    uint16_t src = in8;                     \
    uint16_t dst = mpu::a;                  \
    uint16_t res = dst - src;               \
                                            \
    mpu::zf = (uint8_t) res;                \
    mpu::nf = UINT32_MAX;                   \
    mpu::hf = (dst & 0xF) - (src & 0x0F);   \
    mpu::cf = res;                          \
  } 

#define RLC_S_(opcode)                        \
  (uint8_t ) (t << 1 | t >> 7) 
#define RLC_(opcode)                          \
  { /* ROL */                                 \
    mpu::hf = 0;                              \
    mpu::nf = 0;                              \
    mpu::cf = (uint32_t) in8 << 1 | in8 >> 7; \
    mpu::zf = (uint8_t) mpu::cf ;             \
                                              \
    out8 = (uint8_t) mpu::cf ;                \
  }

#define RRC_S_(opcode)                         \
 (uint8_t ) (t >> 1 | t << 7)                        
#define RRC_(opcode)                            \
  { /* ROR */                                  \
    mpu::zf = (uint8_t) (in8 >> 1 | in8 << 7); \
    mpu::cf = (uint32_t) in8 << 8;             \
    mpu::hf = 0;                               \
    mpu::nf = 0;                               \
                                               \
    out8 = mpu::zf ;                           \
  }

#define RL_S_(opcode)                                        \
 (uint8_t ) ((t << 1 | (mpu::cf >> 8 & 1)))
#define RL_(opcode)                  \
  { /* RCL */                                                \
    mpu::hf = 0;                                             \
    mpu::nf = 0;                                             \
    mpu::cf = ((uint32_t) in8 << 1 | (mpu::cf >> 8 & 1));    \
    mpu::zf = (uint8_t) mpu::cf ;                            \
                                                             \
    out8 = (uint8_t) mpu::cf ;                              \
  }

#define RR_S_(opcode) \
 (uint8_t ) ((t >> 1 | (mpu::cf >> 1 & 0x80)))       
#define RR_(opcode)                                          \
  { /* RCR */                                                \
    mpu::zf = (uint8_t) (in8 >> 1 | (mpu::cf >> 1 & 0x80));  \
    mpu::cf = (uint32_t) in8 << 8;                           \
    mpu::hf = 0;                                             \
    mpu::nf = 0;                                             \
                                                             \
    out8 = mpu::zf ;                                        \
  }

#define SLA_S_(opcode)               \
 (uint8_t ) (t << 1)
#define SLA_(opcode)               \
  { /* SHL */                      \
    mpu::hf = 0;                   \
    mpu::nf = 0;                   \
    mpu::cf = (uint32_t) in8 << 1; \
    mpu::zf = (uint8_t) mpu::cf ;  \
                                   \
    out8 = (uint8_t) mpu::cf ;    \
  }

#define SRA_S_(opcode) \
 (uint8_t ) ((int8_t) t >> 1)
#define SRA_(opcode)                                 \
  { /* SAR */                                        \
    int8_t res = (int8_t) in8 >> 1;                \
                                                      \
    mpu::cf = (uint32_t) in8 << 8;                  \
    mpu::zf = (uint8_t) res;                          \
    mpu::hf = 0;                                      \
    mpu::nf = 0;                                      \
                                                      \
    out8 = (uint8_t) res ;                          \
  }

#define SWAP_S_(opcode) \
 (uint8_t ) (t << 4 | t >> 4)
#define SWAP_(opcode)                          \
  {                                                   \
    mpu::zf = (uint8_t) (in8 >> 4 | in8 << 4);        \
    mpu::cf = 0;                                      \
    mpu::hf = 0;                                      \
    mpu::nf = 0;                                      \
                                                      \
    out8 = mpu::zf ;                                  \
  }

#define SRL_S_(opcode)                \
 (uint8_t ) (t >> 1)
#define SRL_(opcode)                  \
  { /* SHR */                         \
    mpu::cf = (uint32_t) in8 << 8;    \
    mpu::zf = (uint8_t) in8 >> 1;     \
    mpu::hf = 0;                      \
    mpu::nf = 0;                      \
                                      \
    out8 = (uint8_t) mpu::zf;         \
  }

#define BIT_S_(opcode) \
   (uint8_t ) (0)
#define BIT_(opcode)                           \
  {                                            \
    mpu::zf = in8 & 1 << ((opcode >> 3) & 7);  \
    mpu::hf = UINT32_MAX;                      \
    mpu::nf = 0;                               \
  }

#define RES_S_(opcode) \
 (uint8_t ) (t & (uint8_t)~(1 << ((opcode >> 3) & 7)))
#define RES_(opcode)                                    \
  {                                                     \
    out8 = in8 & (uint8_t)~(1 << ((opcode >> 3) & 7));  \
  }

#define SET_S_(opcode) \
 (uint8_t ) (t | (uint8_t) (1 << ((opcode >> 3) & 7)))
#define SET_(opcode)                                    \
  {                                                     \
    out8 = in8 | (uint8_t) (1 << ((opcode >> 3) & 7));  \
  }

#define INC_()                   \
  {                              \
    out8 = in8 + 1;              \
                                 \
    mpu::zf = out8;              \
    mpu::nf = 0;                 \
    mpu::hf = (out8 & 0x0F) - 1; \
  }

#define DEC_()                   \
  {                              \
    out8 = in8 - 1;              \
                                 \
    mpu::zf = out8;              \
    mpu::nf = UINT32_MAX;        \
    mpu::hf = (in8 & 0x0F) - 1;  \
  }

#define ATOM_(opinc_start, reg)                \
  case opinc_start: /* $opcode:INC reg */      \
    in8 = reg;                                 \
                                               \
    INC_ ()                                    \
                                               \
    reg = out8;                                \
                                               \
    OPCL (1, 4)                                \
    break;                                     \
                                               \
  case opinc_start + 1: /* $opcode:DEC reg */  \
    in8 = reg;                                 \
                                               \
    DEC_ ()                                    \
                                               \
    reg = out8;                                \
                                               \
    OPCL (1, 4)                                \
    break;

#define PUSH_(reg)                           \
  WRITE_HOOK2 ( mpu::sp - 2, reg);           \
                                             \
  gb::write (mpu::sp - 1, reg >> 8);         \
  gb::write (mpu::sp - 2, reg & 0xff);       \
                                             \
  mpu::sp--;                                 \
  mpu::sp--;                                 \
                                             \
  OPCL (1, 16)              

#define POP_(h, l)                           \
  READ_HOOK2 (mpu::sp)                       \
                                             \
  l = gb::read (mpu::sp + 0);                \
  h = gb::read (mpu::sp + 1);                \
                                             \
  mpu::sp++;                                 \
  mpu::sp++;                                 \
                                             \
  OPCL (1, 12)                    

#define JR_(opcode, COND)           \
  case opcode:                      \
                                    \
   if ( COND )                      \
   {                                \
     int16_t e = static_cast<int8_t> (OPCODE1);\
                                    \
     mpu::pc += 2;                  \
     mpu::pc += e;                  \
                                    \
     op_ticks = 12;                 \
   }                                \
   else                             \
   {                                \
     OPCL (2, 8)                    \
   }                                \
   break;

#define JP_(opcode, COND)           \
  case opcode:                      \
                                    \
   if ( COND )                      \
   {                                \
     mpu::pcl = OPCODE1;          \
     mpu::pch = OPCODE2;          \
                                    \
     op_ticks = 16;                 \
   }                                \
   else                             \
   {                                \
     OPCL (3, 12)                   \
   }                                \
   break;

#define CALL_(opcode, COND)                     \
  case opcode:                                  \
                                                \
   if ( COND )                                  \
   {                                            \
     uint16_t ppc = mpu::pc;                    \
     uint8_t pcl = OPCODE1;                     \
     uint8_t pch = OPCODE2;                     \
                                                \
     mpu::pc += 3;                              \
                                                \
     gb::write (--mpu::sp, mpu::pch);           \
     gb::write (--mpu::sp, mpu::pcl);           \
                                                \
     mpu::pcl = pcl;                            \
     mpu::pch = pch;                            \
                                                \
     mpu::call_push_rt (mpu::pc, ppc, ppc + 3); \
     op_ticks = 24;                             \
   }                                            \
   else                                         \
   {                                            \
     OPCL (3, 12)                               \
   }                                            \
   break;

#define RET_(opcode, COND, cond_hit_cycle)            \
  case opcode:                                        \
                                                      \
   if ( COND )                                        \
   {                                                  \
     mpu::pcl = gb::read (mpu::sp++);               \
     mpu::pch = gb::read (mpu::sp++);               \
                                                      \
     op_ticks = cond_hit_cycle;                       \
                                                      \
     mpu::call_pop_rt ();                             \
   }                                                  \
   else                                               \
   {                                                  \
     OPCL (1, 8)                                      \
   }                                                  \
                                                      \
   if ( opcode == 0xD9 )                              \
   {                                                  \
     mpu::int_pending&=~0x1f;                         \
     mpu::int_pending|= mpu::if_mask & mpu::ie_mask & 0x1f;  \
     mpu::ime_mask = 0xff;                            \
                                                      \
     mpu::int_pending |= 0x80;                        \
   }                                                  \
   break;

#define RST_(opcode, to)                           \
  case opcode:                                     \
                                                   \
    mpu::call_push_rt (to, mpu::pc, mpu::pc + 1);       \
                                                   \
    mpu::pc++;                              \
                                            \
    gb::write (--mpu::sp, mpu::pch); \
    gb::write (--mpu::sp, mpu::pcl); \
                                            \
    mpu::pc = to;                           \
                                            \
    op_ticks = 16;                     \
    break;

#define LD__(op_start, reg, dst)                \
  case (op_start): /* $opcode:LD dst, B */      \
    dst = reg;                                  \
                                                \
    OPCL (1, 4)                                 \
    break;                                      \

#define LD_(op_start, dst)                                  \
  LD__ (op_start + 0, mpu::b, dst) /* $opcode:LD dst, B */  \
  LD__ (op_start + 1, mpu::c, dst) /* $opcode:LD dst, C */  \
  LD__ (op_start + 2, mpu::d, dst) /* $opcode:LD dst, D */  \
  LD__ (op_start + 3, mpu::e, dst) /* $opcode:LD dst, E */  \
  LD__ (op_start + 4, mpu::h, dst) /* $opcode:LD dst, H */  \
  LD__ (op_start + 5, mpu::l, dst) /* $opcode:LD dst, L */  \
  LD__ (op_start + 7, mpu::a, dst) /* $opcode:LD dst, A */  \
                                                            \
  case (op_start + 6): /* $opcode:LD dst, (HL) */           \
    READ_HOOK (mpu::hl)                                     \
                                                            \
    dst = gb::read (mpu::hl);                               \
                                                            \
    OPCL (1, 8)                                             \
    break;                                        

#define ALU__(op_start, reg, alu)                     \
  case (op_start): /* $opcode:alu dst, B */           \
    in8 = reg;                                        \
                                                      \
    alu()                                             \
                                                      \
    OPCL (1, 4)                                       \
    break;                                            

#define ALU_(op_start, alu)                                   \
  case (op_start + 0x46): /* $opcode:alu dst, d8 */           \
    in8 = OPCODE1;                                            \
                                                              \
    alu()                                                     \
                                                              \
    OPCL (2, 8)                                               \
    break;                                                    \
                                                              \
  ALU__ (op_start + 0, mpu::b, alu) /* $opcode:alu dst, B */  \
  ALU__ (op_start + 1, mpu::c, alu) /* $opcode:alu dst, C */  \
  ALU__ (op_start + 2, mpu::d, alu) /* $opcode:alu dst, D */  \
  ALU__ (op_start + 3, mpu::e, alu) /* $opcode:alu dst, E */  \
  ALU__ (op_start + 4, mpu::h, alu) /* $opcode:alu dst, H */  \
  ALU__ (op_start + 5, mpu::l, alu) /* $opcode:alu dst, L */  \
  ALU__ (op_start + 7, mpu::a, alu) /* $opcode:alu dst, A */  \
                                                              \
  case (op_start + 6): /* $opcode:alu dst, (HL) */            \
    READ_HOOK (mpu::hl)                                       \
                                                              \
    in8 = gb::read (mpu::hl);                                 \
                                                              \
    alu()                                                     \
                                                              \
    OPCL (1, 8)                                               \
    break;                                            

#define ALU2__(op_start, reg, alu, write_back)    \
  case (op_start): /* $opcode:alu dst, B */       \
    in8 = reg;                                    \
                                                  \
    alu ((op_start))                              \
                                                  \
    if ( write_back != 0 )                        \
    {                                             \
      reg = out8;                                 \
    }                                             \
    OPCL (2, 8)                                   \
    break;                                        

#define ALU2_(op_start, alu, write_back, bp_hl_op)                         \
  ALU2__ (op_start + 0, mpu::b, alu, write_back) /* $opcode:alu dst, B */  \
  ALU2__ (op_start + 1, mpu::c, alu, write_back) /* $opcode:alu dst, C */  \
  ALU2__ (op_start + 2, mpu::d, alu, write_back) /* $opcode:alu dst, D */  \
  ALU2__ (op_start + 3, mpu::e, alu, write_back) /* $opcode:alu dst, E */  \
  ALU2__ (op_start + 4, mpu::h, alu, write_back) /* $opcode:alu dst, H */  \
  ALU2__ (op_start + 5, mpu::l, alu, write_back) /* $opcode:alu dst, L */  \
  ALU2__ (op_start + 7, mpu::a, alu, write_back) /* $opcode:alu dst, A */  \
                                                                           \
  case (op_start + 6): /* $opcode:alu dst, (HL) */                         \
    if ( op_start >= 0x40 && op_start <= 0x7f )                            \
    { /* BIT, only read */                                                 \
      READ_HOOK (mpu::hl)                                                  \
    }                                                                      \
    else                                                                   \
    {                                                                      \
      uint8_t t = gb::read2 (mpu::hl);                                     \
                                                                           \
      ACCESS_HOOK (mpu::hl, bp_hl_op ( (op_start + 6)) )                   \
    }                                                                      \
    in8 = gb::read (mpu::hl);                                              \
                                                                           \
    alu((op_start + 6))                                                    \
                                                                           \
    if ( write_back != 0 )                                                 \
    {                                                                      \
      gb::write (mpu::hl, out8);                                           \
    }                                                                      \
    OPCL (2, 16)                                                           \
    break;                                                         

#define ST_(op, src)                  \
  case op: /* $opcode:LD (HL), src */ \
    WRITE_HOOK (mpu::hl, src)         \
                                      \
    gb::write (mpu::hl, src);         \
                                      \
    OPCL (1, 8)                       \
    break; 

#define LD_D16(op, hi, lo) \
  case op:             \
    lo = OPCODE1;      \
    hi = OPCODE2;      \
                       \
    OPCL (3, 12)       \
    break; 

#define ST_xR16(op, reg)        \
  case op:                      \
    WRITE_HOOK (reg, mpu::a)    \
                                \
    gb::write (reg, mpu::a);    \
                                \
    if ( op == 0x32 )           \
    {                           \
      mpu::hl--;                \
    }                           \
    else if ( op == 0x22 )      \
    {                           \
      mpu::hl++;                \
    }                           \
    OPCL (1, 8)                 \
    break;                      

#define LD_xR16(op, reg)       \
  case op:                     \
    READ_HOOK (reg)            \
                               \
    mpu::a = gb::read (reg);   \
                               \
    if ( op == 0x3a )          \
    {                          \
      mpu::hl--;               \
    }                          \
    else if ( op == 0x2a )     \
    {                          \
      mpu::hl++;               \
    }                          \
    OPCL (1, 8)                \
    break;



#if 0
    if ( mpu::pc == 0x4558 && mpu::mmu[4].id == 1 )
    {
      __asm int 3
    }

    if ( mpu::pc == 0x4793 && mpu::mmu[4].id == 1 )
    {
      __asm int 3
    }
        if ( mpu::pc == 0x5dd2 && mpu::mmu[4].id == 1 )
    {
    //  printf ( "------[dba7]--------:%02x \n", mpu::read_directly (0xdba7) );



      __asm int 3

    }
              printf ( "%02x:%04x op:%02x af:%04x bc:%04x de:%04x hl:%04x sp:%04x \n"
        , mpu::mmu[mpu::pc >> 12].id,   mpu::pc, ctr->memory, mpu::af, mpu::bc, mpu::de, mpu::hl, mpu::sp );

    // printf ("%04x:%02x\n", mpu::pc, ctr->memory);
#if 0
    const uint16_t hit[] = 
    {
      0xc344, 0xc347
    , 0xc350, 0xc353
    , 0xc356, 0xc357
    , 0xc35a, 0xc35b
    , 0xc35c, 0xc35e 
    , 0xc360 
    };

    for ( uint32_t id = 0; id != sizeof (hit) / sizeof (hit[0]); id++ )
    {
      if ( hit[id] == mpu::pc )
      {
        printf ("%04x:%02x\n", mpu::pc, ctr->memory);
      }
    }

    if ( mpu::pc == 0xc2c1 )
     __asm int 3
    else if ( mpu::pc == 0xc2ba )
     __asm int 3
#endif 
     // printf ( "pc:%04x op:%02x af:%04x bc:%04x de:%04x hl:%04x sp:%04x \n", mpu::pc, ctr->memory , mpu::af, mpu::bc, mpu::de, mpu::hl, mpu::sp );
    static int sse =0;

    if ( dbg_flags != 0 && mpu::pc == 0x28dd )
    {
     sse = 1;
    }
    static FILE *fd = nullptr;

    if ( mpu::pc == 0x5dd2 && mpu::mmu[4].id == 1 )
    {
    //  printf ( "------[dba7]--------:%02x \n", mpu::read_directly (0xdba7) );



      __asm int 3

    }

    if ( mpu::pc == 0x47d1 && mpu::mmu[4].id == 1 && fd == 0 )
    {
        fd = fopen ("e://aaaaaaaaaaaaaaaaaaaaaaaaaa.cpp", "wb+");

    }

  //if ( fd != 0 )
    {
#if 0
      const uint16_t hit[] = 
    {
      0x28d9, 0x28db, 0x28d7, 0x0372,
      0x2900, 0x2901, 0x2902, 0x2903, 0x2904, 0x2905,
      0x2914, 0x2915, 0x2916, 0x2917, 0x2918, 0x2919, 0x291a 

    };
      bool sec = 0;

    for ( uint32_t id = 0; id != sizeof (hit) / sizeof (hit[0]); id++ )
    {
      if ( hit[id] == mpu::pc )
      {
        sec = 1;
      }
    }
    if ( mpu::pc == 0x2d19 )
      __asm int 3

     if ( sec == 0 )
     {
      fprintf ( fd, "%02x:%04x op:%02x af:%04x bc:%04x de:%04x hl:%04x sp:%04x \n"
         , mpu::mmu[mpu::pc >> 12].id,   mpu::pc, ctr->memory, mpu::af, mpu::bc, mpu::de, mpu::hl, mpu::sp );
      fflush (fd);
     }
#else 
     // printf ( "%02x:%04x op:%02x af:%04x bc:%04x de:%04x hl:%04x sp:%04x \n"
     //    , mpu::mmu[mpu::pc >> 12].id,   mpu::pc, ctr->memory, mpu::af, mpu::bc, mpu::de, mpu::hl, mpu::sp );
#endif 
    }

    if ( mpu::pc == 0x2d19 )
    {
      
      fclose (fd);

      fd = 0;

      __asm int 3
    }
#endif 
      switch (ctr->memory)
      { // https://www.pastraiser.com/mpu/gameboy/gameboy_opcodes.html
        // https://gekkio.fi/files/gb-docs/gbctr.pdf 
      case 0xf9: // $opcode:LD SP, HL
        mpu::sp = mpu::hl;

        OPCL (1, 8)
        break;

      case 0x08:
        t16 = OPCODE1;
        t16|= ((uint16_t) OPCODE2) << 8;

        WRITE_HOOK2 ( t16, mpu::sp )

        gb::write (t16 + 0, mpu::spl);
        gb::write (t16 + 1, mpu::sph);

        OPCL (3, 20)
        break;

      case 0xE9:  // $opcode:JP HL
        mpu::pc = mpu::hl;

        op_ticks = 4;
        break;

      default:
        IASSERT (0);
        break;

      case 0xF3: // $opcode:DI
        mpu::ime_mask = 0;
        mpu::int_pending &= ~0x1f;

        OPCL (1, 4)
        break;

      case 0xFB: // $opcode:EI 
        mpu::int_pending&= ~0x1f;
        mpu::int_pending|= (mpu::ie_mask & mpu::if_mask) & 0x1f;
        mpu::int_pending|= 0x80;

        mpu::ime_mask = UINT32_MAX;

        OPCL (1, 4)
        break;

      case 0x10: // $opcode:STOP 

        if ( gb::get_type () != 0 )
        {
          if ( (mpu::key1 & 1) != 0 )
          {
            mpu::pc += 2;

            op_ticks = 4;
            op_ticks2x = 4 >> mpu::speed_switch;

            timer::div_clk = 0; // reset divider

            mpu::key1 &= 0xFE;
            mpu::key1 ^= 0x80;

            mpu::speed_switch ^= 1;
            goto L2;
          }
          else
          {
            // mpu::int_pending |= 0x04; 
#if          1
            mpu::pc++;
            op_ticks = 4;

            // mpu::state |= 0x10; 
#endif 
          }
        }
        else
        {
         // mpu::int_pending |= 0x04;  
#if          1
          mpu::pc++;
          op_ticks = 4;

          // mpu::state |= 0x10; 
#endif 
        }
        break;

      case 0x27: // $opcode:DAA 

        if ( mpu::nf == 0 )
        { // https://forums.nesdev.org/viewtopic.php?t=15944 
          if ( (mpu::cf & 0x100) || mpu::a > 0x99 )
          {
            mpu::a += 0x60;
            mpu::cf = UINT32_MAX;
          }

          if ( (mpu::hf & 0x10) || ( mpu::a & 0x0f) > 0x09 )
          {
            mpu::a += 0x06;
          }
        }
        else 
        {
          if ( (mpu::cf & 0x100) != 0 )
          {
            mpu::a -= 0x60;
          }

          if ( (mpu::hf & 0x10) != 0 )
          {
            mpu::a -= 0x6;
          }
        }
        mpu::zf = mpu::a;
        mpu::hf = 0;

      case 0x00: // $opcode:NOP 
        OPCL (1, 4)
        break;

      case 0x76: // $opcode:HALT 
        mpu::int_pending |= 0x20;

        mpu::halt_int_rmask = 0x1f;
        mpu::ss_halt_rmask = 0;

        op_ticks = 4;
        break;

      LD_D16 (0x01, mpu::b, mpu::c) // $opcode:LD BC, d16
      LD_D16 (0x11, mpu::d, mpu::e) // $opcode:LD DE, d16
      LD_D16 (0x21, mpu::h, mpu::l) // $opcode:LD HL, d16
      LD_D16 (0x31, mpu::sph, mpu::spl) // $opcode:LD SP, d16

      ATOM16_ (0x03, mpu::bc) // $opcode:INC/DEC BC 
      ATOM16_ (0x13, mpu::de) // $opcode:INC/DEC DE 
      ATOM16_ (0x23, mpu::hl) // $opcode:INC/DEC HL 
      ATOM16_ (0x33, mpu::sp) // $opcode:INC/DEC SP 

      LD_D8_ (0x06, mpu::b) // $opcode:LD B, d8 
      LD_D8_ (0x0E, mpu::c) // $opcode:LD C, d8 
      LD_D8_ (0x16, mpu::d) // $opcode:LD D, d8 
      LD_D8_ (0x1E, mpu::e) // $opcode:LD E, d8 
      LD_D8_ (0x26, mpu::h) // $opcode:LD H, d8 
      LD_D8_ (0x2E, mpu::l) // $opcode:LD L, d8 
      LD_D8_ (0x3E, mpu::a) // $opcode:LD A, d8 

      case 0x36: // $opcode:LD (HL), d8 
        t8 = OPCODE1;

        WRITE_HOOK (mpu::hl, t8)

        gb::write (mpu::hl, t8);

        OPCL (2, 12)
        break;

      ST_xR16 (0x02, mpu::bc) // $opcode:LD (BC), A 
      ST_xR16 (0x12, mpu::de) // $opcode:LD (DE), A 
      ST_xR16 (0x22, mpu::hl) // $opcode:LD (HL+), A 
      ST_xR16 (0x32, mpu::hl) // $opcode:LD (HL-), A 

      LD_xR16 (0x0a, mpu::bc) // $opcode:LD A, (BC)
      LD_xR16 (0x1a, mpu::de) // $opcode:LD A, (DE)
      LD_xR16 (0x2a, mpu::hl) // $opcode:LD A, (HL+)
      LD_xR16 (0x3a, mpu::hl) // $opcode:LD A, (HL-)

      case 0xE0: // $opcode:LD (0xFF00+d8), A 
        t16 = OPCODE1;
        t16|= 0xff00;

        WRITE_HOOK (t16, mpu::a)

        gb::write (t16, mpu::a);

        OPCL (2, 12)
        break;

      case 0xE2: // $opcode:LD (0xFF00+C), A 
        t16 = mpu::c;
        t16|= 0xff00;
#if 0
if ( mpu::pc == 0x4eee &&  (0xFF00+mpu::c>= 0xff1c && 0xFF00+mpu::c <= 0xff1d ) )
  printf ("4eee:%04x:%02x\n",0xFF00+mpu::c, mpu::a );
#endif 
        WRITE_HOOK (t16, mpu::a)

        gb::write (t16, mpu::a);

        OPCL (1, 8)
        break;

      case 0xEA: // $opcode:LD (a16), A 
        t16  = OPCODE2;
        t16<<= 8;
        t16 |= OPCODE1;

        WRITE_HOOK (t16, mpu::a)

        gb::write (t16, mpu::a);

        OPCL (3, 16)
        break;

      case 0xF0: // $opcode:LD A, (0xFF00+d8) 
        t16 = OPCODE1;
        t16|= 0xff00;

        READ_HOOK (t16)

        mpu::a = gb::read (t16);

        OPCL (2, 12)
        break;

      case 0xF2: // $opcode:LD A, (0xFF00+C) 
        t16 = mpu::c;
        t16|= 0xff00;

        READ_HOOK (t16)

        mpu::a = gb::read (t16);

        OPCL (1, 8)
        break;

      case 0xFA: // $opcode:LD A, (a16) 
        t16  = OPCODE2;
        t16<<= 8;
        t16 |= OPCODE1;

        READ_HOOK (t16)

        mpu::a = gb::read (t16);

        OPCL (3, 16)
        break;

      LD_ (0x40, mpu::b)
      LD_ (0x48, mpu::c)
      LD_ (0x50, mpu::d)
      LD_ (0x58, mpu::e)
      LD_ (0x60, mpu::h)
      LD_ (0x68, mpu::l)
      ST_ (0x70, mpu::b)
      ST_ (0x71, mpu::c)
      ST_ (0x72, mpu::d)
      ST_ (0x73, mpu::e)
      ST_ (0x74, mpu::h)
      ST_ (0x75, mpu::l)
      ST_ (0x77, mpu::a)
      LD_ (0x78, mpu::a)

      ATOM_(0x04, mpu::b)
      ATOM_(0x0C, mpu::c)
      ATOM_(0x14, mpu::d)
      ATOM_(0x1C, mpu::e)
      ATOM_(0x24, mpu::h)
      ATOM_(0x2C, mpu::l)
      ATOM_(0x3C, mpu::a)

      case 0x34: // $opcode:INC (HL) 
        ACCESS_HOOK (mpu::hl, (gb::read2 ( mpu::hl) + 1) & 0xff)

        in8 = gb::read (mpu::hl);
        INC_ ()
              gb::write (mpu::hl, out8);
        OPCL (1, 12)
        break;

      case 0x35: // $opcode:DEC (HL) 
        ACCESS_HOOK (mpu::hl, (gb::read2 ( mpu::hl) - 1) & 0xff)

        in8 = gb::read (mpu::hl);
        DEC_ ()
              gb::write (mpu::hl, out8);
        OPCL (1, 12)
        break;

      ALU_(0x80, ADD_)
      ALU_(0x88, ADC_)
      ALU_(0x90, SUB_)
      ALU_(0x98, SBC_)
      ALU_(0xA0, AND_)
      ALU_(0xA8, XOR_)
      ALU_(0xB0, ORA_)
      ALU_(0xB8, CMP_)

      ADD16_ (0x09, mpu::bc) // $opcode:ADD HL, BC 
      ADD16_ (0x19, mpu::de) // $opcode:ADD HL, DE 
      ADD16_ (0x29, mpu::hl) // $opcode:ADD HL, HL 
      ADD16_ (0x39, mpu::sp) // $opcode:ADD HL, SP 
      ADD16_X_ (0xe8, mpu::sp, 16) // $opcode:ADD SP, s8 
      ADD16_X_ (0xf8, mpu::hl, 12) // $opcode:LD HL, SP+s8 

      case 0xC5: PUSH_ (mpu::bc) break; // $opcode:PUSH BC 
      case 0xD5: PUSH_ (mpu::de) break; // $opcode:PUSH DE 
      case 0xE5: PUSH_ (mpu::hl) break; // $opcode:PUSH HL 
      case 0xF5:                        // $opcode:PUSH AF 
        mpu::flag2_regf ();
                 PUSH_ (mpu::af) break;

      case 0xC1: POP_ (mpu::b, mpu::c) break; // $opcode:POP BC 
      case 0xD1: POP_ (mpu::d, mpu::e) break; // $opcode:POP DE 
      case 0xE1: POP_ (mpu::h, mpu::l) break; // $opcode:POP HL 
      case 0xF1: POP_ (mpu::a, mpu::f)        // $opcode:POP AF 
        mpu::regf2_flag ();
        mpu::f &= 0xf0;                break;

      case 0x07:  // RLCA (ROL) 
        mpu::zf = UINT32_MAX;
        mpu::nf = 0;
        mpu::hf = 0;
        mpu::cf = (uint32_t) mpu::a << 1;

        mpu::a = (uint8_t) mpu::cf;
        mpu::a|= mpu::cf >> 8;

        OPCL (1, 4)
        break;

      case 0x0F:  // RRCA (ROR) 
        mpu::zf = UINT32_MAX;
        mpu::nf = 0;
        mpu::hf = 0;
        mpu::cf = (uint32_t) mpu::a << 8;

        mpu::a >>= 1;
        mpu::a |= mpu::cf >> 1;
          
        OPCL (1, 4)
        break;

      case 0x17:  // RLA 
        mpu::zf = UINT32_MAX;
        mpu::nf = 0;
        mpu::hf = 0;
        mpu::cf = ((uint32_t) mpu::a << 1) | (mpu::cf >> 8) & 1;

        mpu::a = (uint8_t) mpu::cf;

        OPCL (1, 4)
        break;

      case 0x1F:  // RRA 
        mpu::zf = mpu::cf; // save previous carry 
        mpu::cf = mpu::a << 8; // fill flag carry bit 

        mpu::a >>= 1; // right shift 
        mpu::a |= mpu::zf >> 1 & 0x80; // with previous carry 

        mpu::zf = UINT32_MAX;
        mpu::nf = 0;
        mpu::hf = 0;

        OPCL (1, 4)
        break;

      case 0x2F:  // CPL 
        mpu::nf = UINT32_MAX;
        mpu::hf = UINT32_MAX;
      
        mpu::a ^= 0xff;

        OPCL (1, 4)
        break;

      case 0x37:  // SCF 
        mpu::nf = 0;
        mpu::hf = 0;
        mpu::cf = UINT32_MAX;

        OPCL (1, 4)
        break;

      case 0x3F:  // CCF 
        mpu::nf = 0;
        mpu::hf = 0;
        mpu::cf^= 0x100;

        OPCL (1, 4)
        break;

      RST_ (0xC7, 0x00)
      RST_ (0xD7, 0x10)
      RST_ (0xE7, 0x20)
      RST_ (0xF7, 0x30)
      RST_ (0xCF, 0x08)
      RST_ (0xDF, 0x18)
      RST_ (0xEF, 0x28)
      RST_ (0xFF, 0x38)

      RET_ (0xC0, COND_NZ, 20)
      RET_ (0xD0, COND_NC, 20)
      RET_ (0xC8, COND_Z, 20)
      RET_ (0xD8, COND_C, 20)
      RET_ (0xC9, COND_ALWAYS, 16)
      RET_ (0xD9, COND_ALWAYS, 16)

      JR_ (0x20, COND_NZ)
      JR_ (0x30, COND_NC)
      JR_ (0x28, COND_Z)
      JR_ (0x38, COND_C)
      JR_ (0x18, COND_ALWAYS)

      JP_ (0xC2, COND_NZ)
      JP_ (0xD2, COND_NC)
      JP_ (0xCA, COND_Z)
      JP_ (0xDA, COND_C)
      JP_ (0xC3, COND_ALWAYS)

      CALL_ (0xC4, COND_NZ)
      CALL_ (0xD4, COND_NC)
      CALL_ (0xCC, COND_Z)
      CALL_ (0xDC, COND_C)
      CALL_ (0xCD, COND_ALWAYS)

      case 0xCB:
        switch (OPCODE1)
        {
        ALU2_ ( 0x00, RLC_, 1, RLC_S_)
        ALU2_ ( 0x08, RRC_, 1, RRC_S_)
        ALU2_ ( 0x10, RL_, 1, RL_S_)
        ALU2_ ( 0x18, RR_, 1, RR_S_)
        ALU2_ ( 0x20, SLA_, 1, SLA_S_)
        ALU2_ ( 0x28, SRA_, 1, SRA_S_)
        ALU2_ ( 0x30, SWAP_, 1, SWAP_S_)
        ALU2_ ( 0x38, SRL_, 1, SRL_S_)

        ALU2_ ( 0x40, BIT_, 0, BIT_S_)
        ALU2_ ( 0x48, BIT_, 0, BIT_S_)
        ALU2_ ( 0x50, BIT_, 0, BIT_S_)
        ALU2_ ( 0x58, BIT_, 0, BIT_S_)
        ALU2_ ( 0x60, BIT_, 0, BIT_S_)
        ALU2_ ( 0x68, BIT_, 0, BIT_S_)
        ALU2_ ( 0x70, BIT_, 0, BIT_S_)
        ALU2_ ( 0x78, BIT_, 0, BIT_S_)

        ALU2_ ( 0x80, RES_, 1, RES_S_)
        ALU2_ ( 0x88, RES_, 1, RES_S_)
        ALU2_ ( 0x90, RES_, 1, RES_S_)
        ALU2_ ( 0x98, RES_, 1, RES_S_)
        ALU2_ ( 0xA0, RES_, 1, RES_S_)
        ALU2_ ( 0xA8, RES_, 1, RES_S_)
        ALU2_ ( 0xB0, RES_, 1, RES_S_)
        ALU2_ ( 0xB8, RES_, 1, RES_S_)

        ALU2_ ( 0xC0, SET_, 1, SET_S_)
        ALU2_ ( 0xC8, SET_, 1, SET_S_)
        ALU2_ ( 0xD0, SET_, 1, SET_S_)
        ALU2_ ( 0xD8, SET_, 1, SET_S_)
        ALU2_ ( 0xE0, SET_, 1, SET_S_)
        ALU2_ ( 0xE8, SET_, 1, SET_S_)
        ALU2_ ( 0xF0, SET_, 1, SET_S_)
        ALU2_ ( 0xF8, SET_, 1, SET_S_)

        default:
          IASSERT (0);
          break;
        }
      }
L1:
      op_ticks2x = op_ticks >> mpu::speed_switch;
L2:
      timer::clk (op_ticks);
       apu::clk (op_ticks2x);
       ppu::clk_(op_ticks2x);

      if ( mpu::int_pending != 0 )
      {
        uint32_t mask = mpu::int_pending;
 
        mpu::int_pending &= ~0xc0;
  
        if ( (mpu::ie_mask & mpu::if_mask & mpu::halt_int_rmask ) != 0 )
        { // resume from halt state
          mpu::halt_int_rmask = 0;
          mpu::ss_halt_rmask = 0x0100;

          mpu::int_pending &= ~0x20;

          mpu::pc++;
        }

        if ( (mask & 0x80 ) == 0 && (mpu::int_pending & 0x20) == 0 )
        {
          switch ( mpu::int_pending )
          {
    #define INT_CASE(d4, d3, d2, d1, d0)                  \
      case d4 << 4 | d3 << 3 | d2 << 2 | d1 << 1 | d0 << 0:

    #define INT_SERVICE(pos)                              \
      gb::write (--mpu::sp, mpu::pch);                  \
      gb::write (--mpu::sp, mpu::pcl);                  \
                                                          \
      mpu::pc = 0x40 + 8 * pos;                           \
                                                          \
      mpu::if_mask &= ~(1 << pos);                        \
      mpu::int_pending = 0;                               \
      mpu::ime_mask = 0;                                  \
                                                          \
      mpu::spin_ticks += 20;                              \
                                                          \
      mpu::call_push_rt (mpu::pc, UINT16_MAX, 0x40 + 8 * pos); 

          INT_CASE (0, 0, 0, 0, 1) // vblank- interrupt 
          INT_CASE (0, 0, 0, 1, 1) // vblank- interrupt 
          INT_CASE (0, 0, 1, 0, 1) // vblank- interrupt 
          INT_CASE (0, 0, 1, 1, 1) // vblank- interrupt 
          INT_CASE (0, 1, 0, 0, 1) // vblank- interrupt 
          INT_CASE (0, 1, 0, 1, 1) // vblank- interrupt 
          INT_CASE (0, 1, 1, 0, 1) // vblank- interrupt 
          INT_CASE (0, 1, 1, 1, 1) // vblank- interrupt 
          INT_CASE (1, 0, 0, 0, 1) // vblank- interrupt 
          INT_CASE (1, 0, 0, 1, 1) // vblank- interrupt 
          INT_CASE (1, 0, 1, 0, 1) // vblank- interrupt 
          INT_CASE (1, 0, 1, 1, 1) // vblank- interrupt 
          INT_CASE (1, 1, 0, 0, 1) // vblank- interrupt 
          INT_CASE (1, 1, 0, 1, 1) // vblank- interrupt 
          INT_CASE (1, 1, 1, 0, 1) // vblank- interrupt 
          INT_CASE (1, 1, 1, 1, 1) // vblank- interrupt 
            INT_SERVICE (0)
            break;

          INT_CASE (0, 0, 0, 1, 0) // stat- interrupt 
          INT_CASE (0, 0, 1, 1, 0) // stat- interrupt 
          INT_CASE (0, 1, 0, 1, 0) // stat- interrupt 
          INT_CASE (0, 1, 1, 1, 0) // stat- interrupt 
          INT_CASE (1, 0, 0, 1, 0) // stat- interrupt 
          INT_CASE (1, 0, 1, 1, 0) // stat- interrupt 
          INT_CASE (1, 1, 0, 1, 0) // stat- interrupt 
          INT_CASE (1, 1, 1, 1, 0) // stat- interrupt 
            INT_SERVICE (1)
            break;

          INT_CASE (0, 0, 1, 0, 0) // timer- interrupt 
          INT_CASE (0, 1, 1, 0, 0) // timer- interrupt 
          INT_CASE (1, 0, 1, 0, 0) // timer- interrupt 
          INT_CASE (1, 1, 1, 0, 0) // timer- interrupt 
            INT_SERVICE (2)
            break;

          INT_CASE (0, 1, 0, 0, 0) // serial- interrupt 
          INT_CASE (1, 1, 0, 0, 0) // serial- interrupt 
            INT_SERVICE (3)
            break;

          INT_CASE (1, 0, 0, 0, 0) // joypad- interrupt 
            INT_SERVICE (4)
            break;

          INT_CASE (0, 0, 0, 0, 0) // nothing 
   #undef INT_CASE
            break;

          default:
            IASSERT (0);
          }
        }

        if ( (mask & mpu::ss_halt_rmask) != 0 )
        { // single-step && (not halt state || resume from halt) 
          gb::state |= 31 << 1;
          return (mask & 0x40) >> 5 | 1;
        }

        if ( (mask & 0x40) != 0 )
        { // frame reach mask 
          return 2;
        }
      }
    }
    return 0;
  }

  IR_NOINLINE
  uint8_t mmu_read_ (uint16_t address)
  {
    apu::reg *nr_reg;

    switch (address >> 12)
    {
    case 0x0F:
      if ( address < 0xfe00 )
      {
    case 0x00: // PROM 0x0000-0x0FFF 
    case 0x01: // PROM 0x1000-0x1FFF 
    case 0x02: // PROM 0x2000-0x2FFF 
    case 0x03: // PROM 0x3000-0x3FFF 
    case 0x04: // PROM 0x4000-0x4FFF 
    case 0x05: // PROM 0x5000-0x5FFF 
    case 0x06: // PROM 0x6000-0x6FFF 
    case 0x07: // PROM 0x7000-0x7FFF 
    case 0x08: // VRAM 0x8000-0x8FFF 
    case 0x09: // VRAM 0x9000-0x9FFF 
    case 0x0C: // WRAM 0xC000-0xCFFF 
    case 0x0D: // WRAM 0xD000-0xDFFF 
    case 0x0E: //      0xE000-0xEFFF 
        return mpu::read_directly (address);
      }
      else
      {
        switch (address &= 0x1FF)
        { // APU register io mapper 
        case 0x110: // NR10 - sweep register 
        case 0x111: // NR11 - sound length/wave pattern duty 
        case 0x112: // NR12 - volume/envelope 
        case 0x113: // NR13 - frequency 
        case 0x114: // NR14 - frequency/control 
          nr_reg = & apu::squ_chans[0].regs[address - 0x110];
          return nr_reg->io | nr_reg->or_mask;
        case 0x115: // NR20 - 
        case 0x116: // NR21 - sound length/wave pattern duty 
        case 0x117: // NR22 - volume/envelope 
        case 0x118: // NR23 - frequency 
        case 0x119: // NR24 - frequency/control 
          nr_reg = & apu::squ_chans[1].regs[address - 0x115];
          return nr_reg->io | nr_reg->or_mask;
        case 0x11A: // NR30 - channel's DAC 
        case 0x11B: // NR31 - channel's length 
        case 0x11C: // NR32 - channel's level 
        case 0x11D: // NR33 - frequency 
        case 0x11E: // NR34 - frequency/control 
          nr_reg = & apu::wav_chan_.regs[address - 0x11a];
          return nr_reg->io | nr_reg->or_mask;
        case 0x11F: // NR40 - 
        case 0x120: // NR41 - sound Length 
        case 0x121: // NR42 - volume/envelope (R/W) 
        case 0x122: // NR43 - polynomial counter (R/W) 
        case 0x123: // NR44 - counter/consecutive 
          nr_reg = & apu::noi_chan_.regs[address - 0x11f];
          return nr_reg->io | nr_reg->or_mask;
        case 0x124: // NR50 - master volume & vin panning 
          return apu::nr50.io | apu::nr50.or_mask;
        case 0x125: // NR51 - selection of sound output terminal 
          return apu::nr51.io | apu::nr51.or_mask;
        case 0x126: // NR52 - sound on/off 
          apu::nr52.io |= 0x7f;
          apu::nr52.io &= ~1 | apu::squ_chans[0].apu::chan::enable_mask;
          apu::nr52.io &= ~2 | apu::squ_chans[1].apu::chan::enable_mask;
          apu::nr52.io &= ~4 | apu::wav_chan_.apu::chan::enable_mask;
          apu::nr52.io &= ~8 | apu::noi_chan_.apu::chan::enable_mask;
          return apu::nr52.io | apu::nr52.or_mask;
        case 0x127:
        case 0x128:
        case 0x129:
        case 0x12A:
        case 0x12B:
        case 0x12C: 
        case 0x12D: 
        case 0x12E: 
        case 0x12F:
          return 0xff;
        case 0x130: case 0x131: case 0x132: case 0x133:
        case 0x134: case 0x135: case 0x136: case 0x137:
        case 0x138: case 0x139: case 0x13A: case 0x13B:
        case 0x13C: case 0x13D: case 0x13E: case 0x13F:
          return apu::wav_ram_read (address);

        case 0x00: case 0x01: case 0x02: case 0x03: 
        case 0x04: case 0x05: case 0x06: case 0x07:
        case 0x08: case 0x09: case 0x0A: case 0x0B:
        case 0x0C: case 0x0D: case 0x0E: case 0x0F:
        case 0x10: case 0x11: case 0x12: case 0x13: 
        case 0x14: case 0x15: case 0x16: case 0x17:
        case 0x18: case 0x19: case 0x1A: case 0x1B:
        case 0x1C: case 0x1D: case 0x1E: case 0x1F:
        case 0x20: case 0x21: case 0x22: case 0x23: 
        case 0x24: case 0x25: case 0x26: case 0x27:
        case 0x28: case 0x29: case 0x2A: case 0x2B:
        case 0x2C: case 0x2D: case 0x2E: case 0x2F:
        case 0x30: case 0x31: case 0x32: case 0x33: 
        case 0x34: case 0x35: case 0x36: case 0x37:
        case 0x38: case 0x39: case 0x3A: case 0x3B:
        case 0x3C: case 0x3D: case 0x3E: case 0x3F:
        case 0x40: case 0x41: case 0x42: case 0x43: 
        case 0x44: case 0x45: case 0x46: case 0x47:
        case 0x48: case 0x49: case 0x4A: case 0x4B:
        case 0x4C: case 0x4D: case 0x4E: case 0x4F:
        case 0x50: case 0x51: case 0x52: case 0x53: 
        case 0x54: case 0x55: case 0x56: case 0x57:
        case 0x58: case 0x59: case 0x5A: case 0x5B:
        case 0x5C: case 0x5D: case 0x5E: case 0x5F:
        case 0x60: case 0x61: case 0x62: case 0x63: 
        case 0x64: case 0x65: case 0x66: case 0x67:
        case 0x68: case 0x69: case 0x6A: case 0x6B:
        case 0x6C: case 0x6D: case 0x6E: case 0x6F:
        case 0x70: case 0x71: case 0x72: case 0x73: 
        case 0x74: case 0x75: case 0x76: case 0x77:
        case 0x78: case 0x79: case 0x7A: case 0x7B:
        case 0x7C: case 0x7D: case 0x7E: case 0x7F:
        case 0x80: case 0x81: case 0x82: case 0x83: 
        case 0x84: case 0x85: case 0x86: case 0x87:
        case 0x88: case 0x89: case 0x8A: case 0x8B:
        case 0x8C: case 0x8D: case 0x8E: case 0x8F:
        case 0x90: case 0x91: case 0x92: case 0x93: 
        case 0x94: case 0x95: case 0x96: case 0x97:
        case 0x98: case 0x99: case 0x9A: case 0x9B:
        case 0x9C: case 0x9D: case 0x9E: case 0x9F:

          return reinterpret_cast<uint8_t *> (& ppu::sprite_[address >> 2])[address & 3];
           
        case 0x1FF:
          return mpu::ie_mask;

        case 0x100:
          return joypad::read ();

        case 0x101:
          return 0xff;
        case 0x102:
          return 0xff;
        case 0x103:
          return 0xff;
        case 0x104:
          return timer::divider_read ();
        case 0x105:
          return timer::tima_read ();
        case 0x106:
          return timer::tma_read ();
        case 0x107:
          return timer::tac_read ();
        case 0x10F:
          return mpu::if_mask;
        case 0x140:
          return ppu::control;
        case 0x141:
          return ppu::stat_read ();
        case 0x142:
          return ppu::scr_y;
        case 0x143:
          return ppu::scr_x;
        case 0x144:
          return static_cast<uint8_t> (ppu::line);
        case 0x145:
          return ppu::ly_cmp;
        case 0x146:
          return ppu::oam_dma;
        case 0x147:
          return ppu::bgp_obp[0];
        case 0x148:
          return ppu::bgp_obp[2];
        case 0x149:
          return ppu::bgp_obp[3];
        case 0x14A:
          return ppu::win_y;
        case 0x14B:
          return ppu::win_x;
        case 0x14D:
          return mpu::key1 | 0x7e;
        case 0x14F:
          if ( get_type () != 0 )
          {
            return ppu::vbk | 0xfe;
          }
          return 0xff;
        case 0x155:
          return static_cast<uint8_t> (ppu::dma_.control);
        case 0x168: // BCPS 
        case 0x16A: // OCPS 
          if ( get_type () != 0 )
          {
            bcp_ocp &bpa = ppu::bcp_ocp_[address >> 1 & 1];

            uint8_t reg = bpa.source;
                    reg|= 0x40;
                    reg|= bpa.access & 0x3f;
            return reg;
          }
          return 0xff;

        case 0x169: // BCPD 
        case 0x16B: // OCPD 
          if ( get_type () != 0 )
          {
            ppu::bcp_ocp &bpa = ppu::bcp_ocp_[address >> 1 & 1];

            uint8_t vl = bpa.pal[bpa.access & 0x3f];

            if ( ( bpa.access & 1 ) != 0 )
            {
              vl |= 0x80;
            }
            return vl;
          }
          return 0xff;

        case 0x170: // SVBK 
          if ( get_type () != 0 )
          {
            return mpu::svbk | 0xf8;
          }
          return 0xff;

        case 0x156: // RP 
          return mpu::io[address].memory;

        default:
          return mpu::io[address].memory;
          break;
        }
      }
      return 0;

    case 0x0A: // ERAM 0xA000-0xAFFF
    case 0x0B: // ERAM 0xB000-0xBFFF 
      return cart::read ( address );
    }
    return 0;
  }

  IR_NOINLINE
  uint8_t mmu_read2_ (struct dmg *dmg, uint16_t address)
  {
    return 0;
  }

  IR_FORCEINLINE  uint8_t 
  read (uint16_t address)
  {
    return ( address < 0x7fff ) 
      ?  mpu::read_directly (address)
    : mmu_read_ (address );
  }

  IR_FORCEINLINE  uint8_t 
  read2 (uint16_t address)
  { // reads without side effects (e.g., causing changes in the internal state of the register)
    return ( address < 0x7fff ) 
      ? mpu::read_directly (address)
    : mmu_read_ (address );
  }

  int 
  reset (void)
  {
    if ( (gb::state & 0x4000) != 0 )
    {
      ppu::reset ();
      apu::reset ();
      joypad::reset ();
      timer::reset ();
      mpu::reset ();
      cart::reset ();

      return 0;
    }
    return -1;
  }

  gb (void):state (0)
  {
  }

  void 
  write (uint16_t address, uint8_t value)
  {
    mpu::addr_map *am;
    nbyte *nb;

    if ( address == 0xdb80)
    {
      printf ( "%02x:%04x [0xdb80]:%02x \n", mpu::mmu[13].id,  mpu::pc, value );
    }

    switch (address >> 12)
    {
    case 0x00: // PROM 0x0000-0x0FFF 
    case 0x01: // PROM 0x1000-0x1FFF 
    case 0x02: // PROM 0x2000-0x2FFF 
    case 0x03: // PROM 0x3000-0x3FFF 
    case 0x04: // PROM 0x4000-0x4FFF 
    case 0x05: // PROM 0x5000-0x5FFF 
    case 0x06: // PROM 0x6000-0x6FFF 
    case 0x07: // PROM 0x7000-0x7FFF 
    case 0x0A: // ERAM 0xA000-0xAFFF 
    case 0x0B: // ERAM 0xB000-0xBFFF 
      cart::write ( address, value );
      break;

    case 0x08: // VRAM 0x8000-0x8FFF 
    case 0x09: // VRAM 0x9000-0x9FFF 

      am = & mpu::mmu[address >> 12];
      nb = & am->ptr[address & 0xfff];
      nb->memory = value;

      if ( get_type () == 0 )
      { // dmg mode vram write
        if ( address <= 0x97FF )
        { // write vram character data
          ppu::dmg_character_discard ( address );
        }
        else
        { // write vram nt_rdc/attribute data
          ppu::dmg_nametable_update (ppu::nt_rdc_[address & 0x7FF], value);
        }   
      }
      else
      {
        if ( address <= 0x97FF )
        { // write vram character data 
          ppu::cgb_character_discard (am->id, address );
        }
        else
        { // write vram nt_rdc/attribute data
          ppu::cgb_nametable_update (ppu::nt_rdc_[address & 0x7FF], & ppu::vram[0x1800 + (address & 0x7ff)]);
        }
      }
      break;

    case 0x0E: // ECHO 0xE000-0xEFFF 
    case 0x0F: // ECHO OR IO 
      if ( address < 0xFE00 )
      {
    case 0x0C: // WRAM 0xC000-0xCFFF
    case 0x0D: // WRAM 0xD000-0xDFFF 
        mpu::write_directly (address, value);
        return ;
      }
      else
      {
        switch (address &= 0x1FF)
        { // APU register io mapper 
        case 0x110: // NR10 - sweep register 
          nr10_write (value);
          break;
        case 0x111: // NR11 - sound length/wave pattern duty 
          nrx1_write ( apu::squ_chans[0], value );
          break;
        case 0x112: // NR12 - volume/envelope 
          nrx2_write ( apu::squ_chans[0], value );
          break;
        case 0x113: // NR13 - frequency 
          nrx3_write ( apu::squ_chans[0], value );
          break;
        case 0x114: // NR14 - frequency/control 
          nrx4_write ( apu::squ_chans[0], value );
          break;
        case 0x115: 
          break;
        case 0x116: // NR21 - sound length/wave pattern duty 
          nrx1_write ( apu::squ_chans[1], value );
          break;
        case 0x117: // NR22 - volume/envelope 
          nrx2_write ( apu::squ_chans[1], value );
          break;
        case 0x118: // NR23 - frequency 
          nrx3_write ( apu::squ_chans[1], value );
          break;
        case 0x119: // NR24 - frequency/control 
          nrx4_write ( apu::squ_chans[1], value );
          break;
        case 0x11A: // NR30 - channel's DAC 
          nr30_write ( value );
          break;
        case 0x11B: // NR31 - channel's length 
          nr31_write ( value );
          break;
        case 0x11C: // NR32 - channel's level 
          nr32_write ( value );
          break;
        case 0x11D: // NR33 - frequency 
          nr33_write ( value );
          break;
        case 0x11E: // NR34 - frequency/control 
          nr34_write ( value );
          break;
        case 0x11F:
          break;
        case 0x120: // NR41 - sound Length 
          nr41_write ( value );
          break;
        case 0x121: // NR42 - volume/envelope (R/W) 
          nr42_write ( value );
          break;
        case 0x122: // NR43 - polynomial counter (R/W) 
          nr43_write ( value );
          break;
        case 0x123: // NR44 - counter/consecutive 
          nr44_write ( value );
          break;
        case 0x124: // NR50 - master volume & vin panning 
          nr50_write ( value );
          break;
        case 0x125: // NR51 - selection of sound output terminal 
          nr51_write ( value );
          break;
        case 0x126: // NR52 - sound on/off 
          nr52_write ( value );
          break;
        case 0x127:
        case 0x128:
        case 0x129:
        case 0x12A:
        case 0x12B:
        case 0x12C: 
        case 0x12D: 
        case 0x12E: 
        case 0x12F:
          break;
        case 0x130: case 0x131: case 0x132: case 0x133:
        case 0x134: case 0x135: case 0x136: case 0x137:
        case 0x138: case 0x139: case 0x13A: case 0x13B:
        case 0x13C: case 0x13D: case 0x13E: case 0x13F:
          wav_ram_write ( address & 15, value );
          break;

        case 0x100: // JOYP 
          joypad::write ( value );
          return ;

        case 0x101: // SB 
        case 0x102: // SC 
          return ;

        case 0x104: // FF04 - DIV 
        
          timer::divider_write (value);
          break;

        case 0x105: // FF05-TIMA 

          timer::tima_write (value);
          break;

        case 0x106: // FF06-TMA 

          timer::tma_write (value);
          break;

        case 0x107: // FF07-TAC 

          timer::tac_write (value);
          break;

        case 0x10F: // FF0F - IF 
          mpu::if_mask = value;

          gb::interrupt_update ();
          break;

        case 0x140: // FF40 - LCDC - LCD Control (R/W) 
          ppu::lcdc_write (value );
          break;

        case 0x141: // FF41 - STAT 
          ppu::stat_write (value );
          break;

        case 0x142:
          ppu::scy_write (value);
          break;

        case 0x143:
          ppu::scx_write (value);
          break;

        case 0x145:
          ppu::ly_cmp = value;
          break;

        case 0x146: // FF46 - DMA - DMA Transfer and Start Address (R/W) 
          ppu::oam_dma_ (value);
          break;

        case 0x147: // 0xff47-BGP 
          ppu::dmg_palette_write ( 0, value );
          ppu::dmg_palette_write ( 1, value );
          break;

        case 0x148: // 0xff48-OBP0 
          ppu::dmg_palette_write ( 2, value );
          break;

        case 0x149: // 0xff49-OBP1 
          ppu::dmg_palette_write ( 3, value );
          break;

        case 0x14A:
          ppu::wy_write (value);
          break;

        case 0x14B:
          ppu::wx_write (value);
          break;

        case 0x14D: 
          mpu::key1 = value;
          break;

        case 0x14F: // VBK

          ppu::vbk_write (value);
          break;

        case 0x151:
          ppu::dma_.src_hi = value;
          break;      

        case 0x152:
          ppu::dma_.src_lo = value;
          break;

        case 0x153:
          ppu::dma_.dst_hi = value;
          break;  

        case 0x154:
          ppu::dma_.dst_lo = value;
          break;

        case 0x155:

          if ( static_cast<int8_t> (value) < 0 )
          { // H-blank DMA 
            if ( (gb::state & 0x8000) != 0 )
            { // reset copy elements
              ppu::dma_.control = value & 0x7F;
            }
            else
            {
              gb::state |= 0x8000;

              ppu::dma_.control = value; 

              if ( ppu::mode == 0 )
              {
                ppu::hbl_dma ();
              }
            }
          }
          else
          { // general-purpose DMA 
            if ( (gb::state & 0x8000) != 0 )
            { // close h-blank DMA
              gb::state &= ~0x8000;

              ppu::dma_.control |= 0x80;
            }
            else
            {
              ppu::gp_dma ();

              ppu::dma_.control = 0xffffffff;
            }
          }
          break;     

        case 0x168: 
          ppu::cgb_palette_access_write (0, value);
          break;

        case 0x169: 

          ppu::cgb_palette_data_write (0, value);
          break;

        case 0x16A: 

          ppu::cgb_palette_access_write (1, value);
          break;
      
        case 0x16B: 

          ppu::cgb_palette_data_write (1, value);
          break;   

        case 0x170: 
          if ( gb::get_type () != 0 )
          {
            mpu::mmu[13].id = value & 7;
            mpu::mmu[13].ptr = & mpu::wram[mpu::mmu[13].id * MEM_4K];

            if ( mpu::mmu[13].id == 0 )
            {
              mpu::mmu[13].id = 1;
              mpu::mmu[13].ptr = & mpu::wram[MEM_4K];
            }
            mpu::mmu[15].id = mpu::mmu[13].id;
            mpu::mmu[15].ptr= mpu::mmu[13].ptr;
          }
          mpu::svbk = value;
          break;

        case 0x1FF:
          mpu::ie_mask = value;

          gb::interrupt_update ();
          break;

        case 0x00: case 0x01: case 0x02: case 0x03: 
        case 0x04: case 0x05: case 0x06: case 0x07:
        case 0x08: case 0x09: case 0x0A: case 0x0B:
        case 0x0C: case 0x0D: case 0x0E: case 0x0F:
        case 0x10: case 0x11: case 0x12: case 0x13: 
        case 0x14: case 0x15: case 0x16: case 0x17:
        case 0x18: case 0x19: case 0x1A: case 0x1B:
        case 0x1C: case 0x1D: case 0x1E: case 0x1F:
        case 0x20: case 0x21: case 0x22: case 0x23: 
        case 0x24: case 0x25: case 0x26: case 0x27:
        case 0x28: case 0x29: case 0x2A: case 0x2B:
        case 0x2C: case 0x2D: case 0x2E: case 0x2F:
        case 0x30: case 0x31: case 0x32: case 0x33: 
        case 0x34: case 0x35: case 0x36: case 0x37:
        case 0x38: case 0x39: case 0x3A: case 0x3B:
        case 0x3C: case 0x3D: case 0x3E: case 0x3F:
        case 0x40: case 0x41: case 0x42: case 0x43: 
        case 0x44: case 0x45: case 0x46: case 0x47:
        case 0x48: case 0x49: case 0x4A: case 0x4B:
        case 0x4C: case 0x4D: case 0x4E: case 0x4F:
        case 0x50: case 0x51: case 0x52: case 0x53: 
        case 0x54: case 0x55: case 0x56: case 0x57:
        case 0x58: case 0x59: case 0x5A: case 0x5B:
        case 0x5C: case 0x5D: case 0x5E: case 0x5F:
        case 0x60: case 0x61: case 0x62: case 0x63: 
        case 0x64: case 0x65: case 0x66: case 0x67:
        case 0x68: case 0x69: case 0x6A: case 0x6B:
        case 0x6C: case 0x6D: case 0x6E: case 0x6F:
        case 0x70: case 0x71: case 0x72: case 0x73: 
        case 0x74: case 0x75: case 0x76: case 0x77:
        case 0x78: case 0x79: case 0x7A: case 0x7B:
        case 0x7C: case 0x7D: case 0x7E: case 0x7F:
        case 0x80: case 0x81: case 0x82: case 0x83: 
        case 0x84: case 0x85: case 0x86: case 0x87:
        case 0x88: case 0x89: case 0x8A: case 0x8B:
        case 0x8C: case 0x8D: case 0x8E: case 0x8F:
        case 0x90: case 0x91: case 0x92: case 0x93: 
        case 0x94: case 0x95: case 0x96: case 0x97:
        case 0x98: case 0x99: case 0x9A: case 0x9B:
        case 0x9C: case 0x9D: case 0x9E: case 0x9F:

          if ( gb::get_type () == 0 )
          { 
            ppu::sprite_update<0> ( address, value);
          }
          else
          {
            ppu::sprite_update<1> ( address, value);
          }
          break;

        default:
          mpu::io[address].memory = value;
          break;
        }
      }
      break;

    default:
      break;
    } 
  }

  IR_FORCEINLINE uint8_t 
  get_type (void) const 
  {
    return static_cast<uint8_t> ( 
              static_cast<uint32_t> (state) );
  }
};

#endif 
