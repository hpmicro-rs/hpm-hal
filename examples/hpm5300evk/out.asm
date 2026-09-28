warning: associated functions `set_ahb_clock` and `set_ana_clock` are never used
   --> /Users/mono/Elec/HPM/hpm-hal/src/sysctl/mod.rs:130:8
    |
116 | pub(crate) trait SealedAnalogClockPeripheral {
    |                  --------------------------- associated functions in this trait
...
130 |     fn set_ahb_clock();
    |        ^^^^^^^^^^^^^
131 |
132 |     fn set_ana_clock(cfg: ClockConfig);
    |        ^^^^^^^^^^^^^
    |
    = note: `#[warn(dead_code)]` on by default

warning: associated function `state` is never used
   --> /Users/mono/Elec/HPM/hpm-hal/src/adc/mod.rs:351:8
    |
347 | trait SealedInstance {
    |       -------------- associated function in this trait
...
351 |     fn state() -> &'static State;
    |        ^^^^^

warning: field `state` is never read
   --> /Users/mono/Elec/HPM/hpm-hal/src/dac.rs:181:5
    |
179 | pub struct Dac<'d, M: Mode> {
    |            --- field in this struct
180 |     info: &'static Info,
181 |     state: &'static State,
    |     ^^^^^

warning: unstable feature specified for `-Ctarget-feature`: `d`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `f`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unused variable: `stop_var`
   --> /Users/mono/Elec/embedded-drivers/vl53l0x/src/lib.rs:157:13
    |
157 |         let stop_var = self.read_reg(0x91)?;
    |             ^^^^^^^^ help: if this is intentional, prefix it with an underscore: `_stop_var`
    |
    = note: `#[warn(unused_variables)]` on by default

warning: unstable feature specified for `-Ctarget-feature`: `d`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `f`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `d`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `f`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `d`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `f`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `d`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `f`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `d`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `f`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `d`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `f`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `d`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `f`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unnecessary parentheses around assigned value
  --> /Users/mono/Elec/embedded-drivers/sht20/src/blocking.rs:23:24
   |
23 |         let mut bits = (resolution as u8);
   |                        ^                ^
   |
   = note: `#[warn(unused_parens)]` on by default
help: remove these parentheses
   |
23 -         let mut bits = (resolution as u8);
23 +         let mut bits = resolution as u8;
   |

warning: unnecessary parentheses around assigned value
  --> /Users/mono/Elec/embedded-drivers/sht20/src/lib.rs:62:24
   |
62 |         let mut bits = (resolution as u8);
   |                        ^                ^
   |
help: remove these parentheses
   |
62 -         let mut bits = (resolution as u8);
62 +         let mut bits = resolution as u8;
   |

warning: unused implementer of `Future` that must be used
  --> /Users/mono/Elec/embedded-drivers/sht20/src/lib.rs:74:9
   |
74 |         delay.delay_ms(100);
   |         ^^^^^^^^^^^^^^^^^^^
   |
   = note: futures do nothing unless you `.await` or poll them
   = note: `#[warn(unused_must_use)]` on by default

warning: unused implementer of `Future` that must be used
  --> /Users/mono/Elec/embedded-drivers/sht20/src/lib.rs:88:9
   |
88 |         delay.delay_ms(40);
   |         ^^^^^^^^^^^^^^^^^^
   |
   = note: futures do nothing unless you `.await` or poll them

warning: unstable feature specified for `-Ctarget-feature`: `d`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `f`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `d`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `f`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unused import: `embedded_hal::delay::DelayNs`
 --> src/bin/fuck.rs:7:5
  |
7 | use embedded_hal::delay::DelayNs;
  |     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  |
  = note: `#[warn(unused_imports)]` on by default

warning: unused imports: `Level`, `Output`, and `Speed`
 --> src/bin/fuck.rs:8:17
  |
8 | use hal::gpio::{Level, Output, Speed};
  |                 ^^^^^  ^^^^^^  ^^^^^

warning: unused import: `hal::pac`
 --> src/bin/fuck.rs:9:5
  |
9 | use hal::pac;
  |     ^^^^^^^^

warning: unused import: `riscv::delay::McycleDelay`
  --> src/bin/fuck.rs:10:5
   |
10 | use riscv::delay::McycleDelay;
   |     ^^^^^^^^^^^^^^^^^^^^^^^^^

warning: value assigned to `tmp` is never read
  --> src/bin/fuck.rs:32:13
   |
32 |     let mut tmp: u32 = 0;
   |             ^^^
   |
   = help: maybe it is overwritten before being read?
   = note: `#[warn(unused_assignments)]` on by default

warning: unstable feature specified for `-Ctarget-feature`: `d`
  |
  = note: this feature is not stably supported; its behavior can change in the future

warning: unstable feature specified for `-Ctarget-feature`: `f`
  |
  = note: this feature is not stably supported; its behavior can change in the future


fuck:	file format elf32-littleriscv

Disassembly of section .start:

80003000 <_start>:
80003000: 8007e197     	auipc	gp, 0x8007e
80003004: 80018193     	addi	gp, gp, -0x800

80003008 <.Lpcrel_hi1>:
80003008: 8009d317     	auipc	t1, 0x8009d
8000300c: ff830313     	addi	t1, t1, -0x8
80003010: ff030113     	addi	sp, t1, -0x10
80003014: 00000097     	auipc	ra, 0x0
80003018: 020080e7     	jalr	0x20(ra) <__pre_init>

8000301c <.Lpcrel_hi2>:
8000301c: 8009d317     	auipc	t1, 0x8009d
80003020: fe430313     	addi	t1, t1, -0x1c
80003024: ff030113     	addi	sp, t1, -0x10
80003028: 00001097     	auipc	ra, 0x1
8000302c: d14080e7     	jalr	-0x2ec(ra) <_start_rust>
80003030: 0000006f     	j	0x80003030 <.Lpcrel_hi2+0x14>

80003034 <__pre_init>:
80003034: 00008067     	ret

80003038 <_mp_hook>:
80003038: 00050663     	beqz	a0, 0x80003044 <_mp_hook+0xc>
8000303c: 10500073     	wfi
80003040: ffdff06f     	j	0x8000303c <_mp_hook+0x4>
80003044: 00100513     	li	a0, 0x1
80003048: 00008067     	ret

Disassembly of section .vectors:

00000000 <__vector_ram_start__>:
       0: 0124         	addi	s1, sp, 0x88
       2: 0000         	unimp
       4: 0334         	addi	a3, sp, 0x188
       6: 0000         	unimp
       8: 044c         	addi	a1, sp, 0x204
       a: 0000         	unimp
       c: 3cc2         	fld	fs9, 0x30(sp)
       e: 8000         	<unknown>
      10: 0564         	addi	s1, sp, 0x28c
      12: 0000         	unimp
      14: 3cc2         	fld	fs9, 0x30(sp)
      16: 8000         	<unknown>
      18: 3cc2         	fld	fs9, 0x30(sp)
      1a: 8000         	<unknown>
      1c: 3cc2         	fld	fs9, 0x30(sp)
      1e: 8000         	<unknown>
      20: 3cc2         	fld	fs9, 0x30(sp)
      22: 8000         	<unknown>
		...
      34: 3cc2         	fld	fs9, 0x30(sp)
      36: 8000         	<unknown>
      38: 3cc2         	fld	fs9, 0x30(sp)
      3a: 8000         	<unknown>
      3c: 3cc2         	fld	fs9, 0x30(sp)
      3e: 8000         	<unknown>
      40: 3cc2         	fld	fs9, 0x30(sp)
      42: 8000         	<unknown>
      44: 3cc2         	fld	fs9, 0x30(sp)
      46: 8000         	<unknown>
      48: 3cc2         	fld	fs9, 0x30(sp)
      4a: 8000         	<unknown>
      4c: 3cc2         	fld	fs9, 0x30(sp)
      4e: 8000         	<unknown>
      50: 3cc2         	fld	fs9, 0x30(sp)
      52: 8000         	<unknown>
      54: 3cc2         	fld	fs9, 0x30(sp)
      56: 8000         	<unknown>
      58: 3cc2         	fld	fs9, 0x30(sp)
      5a: 8000         	<unknown>
      5c: 3cc2         	fld	fs9, 0x30(sp)
      5e: 8000         	<unknown>
      60: 3cc2         	fld	fs9, 0x30(sp)
      62: 8000         	<unknown>
      64: 3cc2         	fld	fs9, 0x30(sp)
      66: 8000         	<unknown>
      68: 3cc2         	fld	fs9, 0x30(sp)
      6a: 8000         	<unknown>
      6c: 3cc2         	fld	fs9, 0x30(sp)
      6e: 8000         	<unknown>
      70: 3cc2         	fld	fs9, 0x30(sp)
      72: 8000         	<unknown>
      74: 3cc2         	fld	fs9, 0x30(sp)
      76: 8000         	<unknown>
      78: 3cc2         	fld	fs9, 0x30(sp)
      7a: 8000         	<unknown>
      7c: 3cc2         	fld	fs9, 0x30(sp)
      7e: 8000         	<unknown>
      80: 3cc2         	fld	fs9, 0x30(sp)
      82: 8000         	<unknown>
      84: 3cc2         	fld	fs9, 0x30(sp)
      86: 8000         	<unknown>
      88: 01b0         	addi	a2, sp, 0xc8
      8a: 0000         	unimp
      8c: 3cc2         	fld	fs9, 0x30(sp)
      8e: 8000         	<unknown>
      90: 3cc2         	fld	fs9, 0x30(sp)
      92: 8000         	<unknown>
      94: 3cc2         	fld	fs9, 0x30(sp)
      96: 8000         	<unknown>
      98: 3cc2         	fld	fs9, 0x30(sp)
      9a: 8000         	<unknown>
      9c: 3cc2         	fld	fs9, 0x30(sp)
      9e: 8000         	<unknown>
      a0: 3cc2         	fld	fs9, 0x30(sp)
      a2: 8000         	<unknown>
      a4: 3cc2         	fld	fs9, 0x30(sp)
      a6: 8000         	<unknown>
      a8: 3cc2         	fld	fs9, 0x30(sp)
      aa: 8000         	<unknown>
      ac: 3cc2         	fld	fs9, 0x30(sp)
      ae: 8000         	<unknown>
      b0: 3cc2         	fld	fs9, 0x30(sp)
      b2: 8000         	<unknown>
      b4: 3cc2         	fld	fs9, 0x30(sp)
      b6: 8000         	<unknown>
      b8: 3cc2         	fld	fs9, 0x30(sp)
      ba: 8000         	<unknown>
      bc: 3cc2         	fld	fs9, 0x30(sp)
      be: 8000         	<unknown>
      c0: 3cc2         	fld	fs9, 0x30(sp)
      c2: 8000         	<unknown>
      c4: 3cc2         	fld	fs9, 0x30(sp)
      c6: 8000         	<unknown>
      c8: 3cc2         	fld	fs9, 0x30(sp)
      ca: 8000         	<unknown>
      cc: 3cc2         	fld	fs9, 0x30(sp)
      ce: 8000         	<unknown>
      d0: 3cc2         	fld	fs9, 0x30(sp)
      d2: 8000         	<unknown>
      d4: 3cc2         	fld	fs9, 0x30(sp)
      d6: 8000         	<unknown>
      d8: 3cc2         	fld	fs9, 0x30(sp)
      da: 8000         	<unknown>
      dc: 3cc2         	fld	fs9, 0x30(sp)
      de: 8000         	<unknown>
      e0: 3cc2         	fld	fs9, 0x30(sp)
      e2: 8000         	<unknown>
      e4: 3cc2         	fld	fs9, 0x30(sp)
      e6: 8000         	<unknown>
      e8: 3cc2         	fld	fs9, 0x30(sp)
      ea: 8000         	<unknown>
      ec: 3cc2         	fld	fs9, 0x30(sp)
      ee: 8000         	<unknown>
      f0: 3cc2         	fld	fs9, 0x30(sp)
      f2: 8000         	<unknown>
      f4: 3cc2         	fld	fs9, 0x30(sp)
      f6: 8000         	<unknown>
      f8: 3cc2         	fld	fs9, 0x30(sp)
      fa: 8000         	<unknown>
      fc: 3cc2         	fld	fs9, 0x30(sp)
      fe: 8000         	<unknown>
     100: 3cc2         	fld	fs9, 0x30(sp)
     102: 8000         	<unknown>
     104: 3cc2         	fld	fs9, 0x30(sp)
     106: 8000         	<unknown>
     108: 3cc2         	fld	fs9, 0x30(sp)
     10a: 8000         	<unknown>
     10c: 3cc2         	fld	fs9, 0x30(sp)
     10e: 8000         	<unknown>
     110: 3cc2         	fld	fs9, 0x30(sp)
     112: 8000         	<unknown>
     114: 3cc2         	fld	fs9, 0x30(sp)
     116: 8000         	<unknown>
     118: 3cc2         	fld	fs9, 0x30(sp)
     11a: 8000         	<unknown>
     11c: 3cc2         	fld	fs9, 0x30(sp)
     11e: 8000         	<unknown>
     120: 3cc2         	fld	fs9, 0x30(sp)
     122: 8000         	<unknown>

00000124 <CORE_LOCAL>:
     124: 7139         	addi	sp, sp, -0x40
     126: c006         	sw	ra, 0x0(sp)
     128: c216         	sw	t0, 0x4(sp)
     12a: c41a         	sw	t1, 0x8(sp)
     12c: c61e         	sw	t2, 0xc(sp)
     12e: c872         	sw	t3, 0x10(sp)
     130: ca76         	sw	t4, 0x14(sp)
     132: cc7a         	sw	t5, 0x18(sp)
     134: ce7e         	sw	t6, 0x1c(sp)
     136: d02a         	sw	a0, 0x20(sp)
     138: d22e         	sw	a1, 0x24(sp)
     13a: d432         	sw	a2, 0x28(sp)
     13c: d636         	sw	a3, 0x2c(sp)
     13e: d83a         	sw	a4, 0x30(sp)
     140: da3e         	sw	a5, 0x34(sp)
     142: dc42         	sw	a6, 0x38(sp)
     144: de46         	sw	a7, 0x3c(sp)
     146: 850a         	mv	a0, sp
     148: 534000ef     	jal	0x67c <.Lline_table_start15+0x17>
     14c: 4082         	lw	ra, 0x0(sp)
     14e: 4292         	lw	t0, 0x4(sp)
     150: 4322         	lw	t1, 0x8(sp)
     152: 43b2         	lw	t2, 0xc(sp)
     154: 4e42         	lw	t3, 0x10(sp)
     156: 4ed2         	lw	t4, 0x14(sp)
     158: 4f62         	lw	t5, 0x18(sp)
     15a: 4ff2         	lw	t6, 0x1c(sp)
     15c: 5502         	lw	a0, 0x20(sp)
     15e: 5592         	lw	a1, 0x24(sp)
     160: 5622         	lw	a2, 0x28(sp)
     162: 56b2         	lw	a3, 0x2c(sp)
     164: 5742         	lw	a4, 0x30(sp)
     166: 57d2         	lw	a5, 0x34(sp)
     168: 5862         	lw	a6, 0x38(sp)
     16a: 58f2         	lw	a7, 0x3c(sp)
     16c: 6121         	addi	sp, sp, 0x40
     16e: 30200073     	mret

00000172 <MachineTimer>:
     172: 1141         	addi	sp, sp, -0x10
     174: c606         	sw	ra, 0xc(sp)
     176: c422         	sw	s0, 0x8(sp)
     178: 08000513     	li	a0, 0x80
     17c: 30453073     	csrc	mie, a0
     180: 30047573     	csrrci	a0, mstatus, 0x8
     184: 00857413     	andi	s0, a0, 0x8
     188: 000805b7     	lui	a1, 0x80
     18c: 03858513     	addi	a0, a1, 0x38
     190: 4510         	lw	a2, 0x8(a0)
     192: 4548         	lw	a0, 0xc(a0)
     194: 56fd         	li	a3, -0x1
     196: 02d5ae23     	sw	a3, 0x3c(a1)
     19a: 02d5ac23     	sw	a3, 0x38(a1)
     19e: 9602         	jalr	a2
     1a0: c401         	beqz	s0, 0x1a8 <.Lline_table_start0+0x1a8>
     1a2: 4521         	li	a0, 0x8
     1a4: 30052073     	csrs	mstatus, a0
     1a8: 40b2         	lw	ra, 0xc(sp)
     1aa: 4422         	lw	s0, 0x8(sp)
     1ac: 0141         	addi	sp, sp, 0x10
     1ae: 8082         	ret

000001b0 <HDMA>:
     1b0: 710d         	addi	sp, sp, -0x160
     1b2: 14112e23     	sw	ra, 0x15c(sp)
     1b6: 14512c23     	sw	t0, 0x158(sp)
     1ba: 14612a23     	sw	t1, 0x154(sp)
     1be: 14712823     	sw	t2, 0x150(sp)
     1c2: 14812623     	sw	s0, 0x14c(sp)
     1c6: 0200         	addi	s0, sp, 0x100
     1c8: c424         	sw	s1, 0x48(s0)
     1ca: c068         	sw	a0, 0x44(s0)
     1cc: c02c         	sw	a1, 0x40(s0)
     1ce: dc50         	sw	a2, 0x3c(s0)
     1d0: dc14         	sw	a3, 0x38(s0)
     1d2: d858         	sw	a4, 0x34(s0)
     1d4: d81c         	sw	a5, 0x30(s0)
     1d6: 03042623     	sw	a6, 0x2c(s0)
     1da: 03142423     	sw	a7, 0x28(s0)
     1de: 03242223     	sw	s2, 0x24(s0)
     1e2: 03c42023     	sw	t3, 0x20(s0)
     1e6: 01d42e23     	sw	t4, 0x1c(s0)
     1ea: 01e42c23     	sw	t5, 0x18(s0)
     1ee: 01f42a23     	sw	t6, 0x14(s0)
     1f2: a602         	fsd	ft0, 0x108(sp)
     1f4: a206         	fsd	ft1, 0x100(sp)
     1f6: bd8a         	fsd	ft2, 0xf8(sp)
     1f8: b98e         	fsd	ft3, 0xf0(sp)
     1fa: b592         	fsd	ft4, 0xe8(sp)
     1fc: b196         	fsd	ft5, 0xe0(sp)
     1fe: ad9a         	fsd	ft6, 0xd8(sp)
     200: a99e         	fsd	ft7, 0xd0(sp)
     202: a5a2         	fsd	fs0, 0xc8(sp)
     204: a1a6         	fsd	fs1, 0xc0(sp)
     206: bd2a         	fsd	fa0, 0xb8(sp)
     208: b92e         	fsd	fa1, 0xb0(sp)
     20a: b532         	fsd	fa2, 0xa8(sp)
     20c: b136         	fsd	fa3, 0xa0(sp)
     20e: ad3a         	fsd	fa4, 0x98(sp)
     210: a93e         	fsd	fa5, 0x90(sp)
     212: a542         	fsd	fa6, 0x88(sp)
     214: a146         	fsd	fa7, 0x80(sp)
     216: bcca         	fsd	fs2, 0x78(sp)
     218: b8ce         	fsd	fs3, 0x70(sp)
     21a: b4d2         	fsd	fs4, 0x68(sp)
     21c: b0d6         	fsd	fs5, 0x60(sp)
     21e: acda         	fsd	fs6, 0x58(sp)
     220: a8de         	fsd	fs7, 0x50(sp)
     222: a4e2         	fsd	fs8, 0x48(sp)
     224: a0e6         	fsd	fs9, 0x40(sp)
     226: bc6a         	fsd	fs10, 0x38(sp)
     228: b86e         	fsd	fs11, 0x30(sp)
     22a: b472         	fsd	ft8, 0x28(sp)
     22c: b076         	fsd	ft9, 0x20(sp)
     22e: ac7a         	fsd	ft10, 0x18(sp)
     230: a87e         	fsd	ft11, 0x10(sp)
     232: f00c85b7     	lui	a1, 0xf00c8
     236: 51c8         	lw	a0, 0x24(a1)
     238: 5590         	lw	a2, 0x28(a1)
     23a: 5998         	lw	a4, 0x30(a1)
     23c: 55d4         	lw	a3, 0x2c(a1)
     23e: e76d         	bnez	a4, 0x328 <.Lline_table_start0+0x328>
     240: c111         	beqz	a0, 0x244 <.Lline_table_start0+0x244>
     242: d1c8         	sw	a0, 0x24(a1)
     244: c211         	beqz	a2, 0x248 <.Lline_table_start0+0x248>
     246: d590         	sw	a2, 0x28(a1)
     248: c291         	beqz	a3, 0x24c <.Lline_table_start0+0x24c>
     24a: d5d4         	sw	a3, 0x2c(a1)
     24c: 8d51         	or	a0, a0, a2
     24e: 8d55         	or	a0, a0, a3
     250: c62a         	sw	a0, 0xc(sp)
     252: 497d         	li	s2, 0x1f
     254: 44b1         	li	s1, 0xc
     256: 00080437     	lui	s0, 0x80
     25a: 55840413     	addi	s0, s0, 0x558
     25e: 0068         	addi	a0, sp, 0xc
     260: 80004097     	auipc	ra, 0x80004
     264: a28080e7     	jalr	-0x5d8(ra) <<hpm_hal::internal::BitIter as core::iter::traits::iterator::Iterator>::next::h52a2ccad76180163>
     268: c919         	beqz	a0, 0x27e <.Lline_table_start0+0x27e>
     26a: 0ab96b63     	bltu	s2, a1, 0x320 <.Lline_table_start0+0x320>
     26e: 02958533     	mul	a0, a1, s1
     272: 9522         	add	a0, a0, s0
     274: 80004097     	auipc	ra, 0x80004
     278: 938080e7     	jalr	-0x6c8(ra) <embassy_sync::waitqueue::atomic_waker::AtomicWaker::wake::h356b0cfad1e8b1a3>
     27c: b7cd         	j	0x25e <.Lline_table_start0+0x25e>
     27e: e4200537     	lui	a0, 0xe4200
     282: 414c         	lw	a1, 0x4(a0)
     284: c005f593     	andi	a1, a1, -0x400
     288: 02258593     	addi	a1, a1, 0x22
     28c: c14c         	sw	a1, 0x4(a0)
     28e: 15c12083     	lw	ra, 0x15c(sp)
     292: 15812283     	lw	t0, 0x158(sp)
     296: 15412303     	lw	t1, 0x154(sp)
     29a: 15012383     	lw	t2, 0x150(sp)
     29e: 14c12403     	lw	s0, 0x14c(sp)
     2a2: 14812483     	lw	s1, 0x148(sp)
     2a6: 14412503     	lw	a0, 0x144(sp)
     2aa: 14012583     	lw	a1, 0x140(sp)
     2ae: 13c12603     	lw	a2, 0x13c(sp)
     2b2: 13812683     	lw	a3, 0x138(sp)
     2b6: 13412703     	lw	a4, 0x134(sp)
     2ba: 13012783     	lw	a5, 0x130(sp)
     2be: 12c12803     	lw	a6, 0x12c(sp)
     2c2: 12812883     	lw	a7, 0x128(sp)
     2c6: 12412903     	lw	s2, 0x124(sp)
     2ca: 12012e03     	lw	t3, 0x120(sp)
     2ce: 11c12e83     	lw	t4, 0x11c(sp)
     2d2: 11812f03     	lw	t5, 0x118(sp)
     2d6: 11412f83     	lw	t6, 0x114(sp)
     2da: 2032         	fld	ft0, 0x108(sp)
     2dc: 2092         	fld	ft1, 0x100(sp)
     2de: 316e         	fld	ft2, 0xf8(sp)
     2e0: 31ce         	fld	ft3, 0xf0(sp)
     2e2: 322e         	fld	ft4, 0xe8(sp)
     2e4: 328e         	fld	ft5, 0xe0(sp)
     2e6: 236e         	fld	ft6, 0xd8(sp)
     2e8: 23ce         	fld	ft7, 0xd0(sp)
     2ea: 242e         	fld	fs0, 0xc8(sp)
     2ec: 248e         	fld	fs1, 0xc0(sp)
     2ee: 356a         	fld	fa0, 0xb8(sp)
     2f0: 35ca         	fld	fa1, 0xb0(sp)
     2f2: 362a         	fld	fa2, 0xa8(sp)
     2f4: 368a         	fld	fa3, 0xa0(sp)
     2f6: 276a         	fld	fa4, 0x98(sp)
     2f8: 27ca         	fld	fa5, 0x90(sp)
     2fa: 282a         	fld	fa6, 0x88(sp)
     2fc: 288a         	fld	fa7, 0x80(sp)
     2fe: 3966         	fld	fs2, 0x78(sp)
     300: 39c6         	fld	fs3, 0x70(sp)
     302: 3a26         	fld	fs4, 0x68(sp)
     304: 3a86         	fld	fs5, 0x60(sp)
     306: 2b66         	fld	fs6, 0x58(sp)
     308: 2bc6         	fld	fs7, 0x50(sp)
     30a: 2c26         	fld	fs8, 0x48(sp)
     30c: 2c86         	fld	fs9, 0x40(sp)
     30e: 3d62         	fld	fs10, 0x38(sp)
     310: 3dc2         	fld	fs11, 0x30(sp)
     312: 3e22         	fld	ft8, 0x28(sp)
     314: 3e82         	fld	ft9, 0x20(sp)
     316: 2f62         	fld	ft10, 0x18(sp)
     318: 2fc2         	fld	ft11, 0x10(sp)
     31a: 6135         	addi	sp, sp, 0x160
     31c: 30200073     	mret
     320: 80003097     	auipc	ra, 0x80003
     324: 592080e7     	jalr	0x592(ra) <core::panicking::panic_const::panic_const_div_by_zero::h4fc2a9d0290dad64>
     328: 0305a003     	lw	zero, 0x30(a1)
     32c: 80003097     	auipc	ra, 0x80003
     330: 55e080e7     	jalr	0x55e(ra) <core::panicking::panic_fmt::h525aebbf60a29a7a>

00000334 <GPIO0_A>:
     334: 7129         	addi	sp, sp, -0x140
     336: 12112e23     	sw	ra, 0x13c(sp)
     33a: 12512c23     	sw	t0, 0x138(sp)
     33e: 12612a23     	sw	t1, 0x134(sp)
     342: 12712823     	sw	t2, 0x130(sp)
     346: 12a12623     	sw	a0, 0x12c(sp)
     34a: 0208         	addi	a0, sp, 0x100
     34c: d50c         	sw	a1, 0x28(a0)
     34e: d150         	sw	a2, 0x24(a0)
     350: d114         	sw	a3, 0x20(a0)
     352: cd58         	sw	a4, 0x1c(a0)
     354: cd1c         	sw	a5, 0x18(a0)
     356: 01052a23     	sw	a6, 0x14(a0)
     35a: 01152823     	sw	a7, 0x10(a0)
     35e: 01c52623     	sw	t3, 0xc(a0)
     362: 01d52423     	sw	t4, 0x8(a0)
     366: 01e52223     	sw	t5, 0x4(a0)
     36a: 01f52023     	sw	t6, 0x0(a0)
     36e: bd82         	fsd	ft0, 0xf8(sp)
     370: b986         	fsd	ft1, 0xf0(sp)
     372: b58a         	fsd	ft2, 0xe8(sp)
     374: b18e         	fsd	ft3, 0xe0(sp)
     376: ad92         	fsd	ft4, 0xd8(sp)
     378: a996         	fsd	ft5, 0xd0(sp)
     37a: a59a         	fsd	ft6, 0xc8(sp)
     37c: a19e         	fsd	ft7, 0xc0(sp)
     37e: bd22         	fsd	fs0, 0xb8(sp)
     380: b926         	fsd	fs1, 0xb0(sp)
     382: b52a         	fsd	fa0, 0xa8(sp)
     384: b12e         	fsd	fa1, 0xa0(sp)
     386: ad32         	fsd	fa2, 0x98(sp)
     388: a936         	fsd	fa3, 0x90(sp)
     38a: a53a         	fsd	fa4, 0x88(sp)
     38c: a13e         	fsd	fa5, 0x80(sp)
     38e: bcc2         	fsd	fa6, 0x78(sp)
     390: b8c6         	fsd	fa7, 0x70(sp)
     392: b4ca         	fsd	fs2, 0x68(sp)
     394: b0ce         	fsd	fs3, 0x60(sp)
     396: acd2         	fsd	fs4, 0x58(sp)
     398: a8d6         	fsd	fs5, 0x50(sp)
     39a: a4da         	fsd	fs6, 0x48(sp)
     39c: a0de         	fsd	fs7, 0x40(sp)
     39e: bc62         	fsd	fs8, 0x38(sp)
     3a0: b866         	fsd	fs9, 0x30(sp)
     3a2: b46a         	fsd	fs10, 0x28(sp)
     3a4: b06e         	fsd	fs11, 0x20(sp)
     3a6: ac72         	fsd	ft8, 0x18(sp)
     3a8: a876         	fsd	ft9, 0x10(sp)
     3aa: a47a         	fsd	ft10, 0x8(sp)
     3ac: a07e         	fsd	ft11, 0x0(sp)
     3ae: 4501         	li	a0, 0x0
     3b0: 80004097     	auipc	ra, 0x80004
     3b4: 878080e7     	jalr	-0x788(ra) <hpm_hal::gpio::input_future::on_interrupt::hcc22a711960b534d>
     3b8: e4200537     	lui	a0, 0xe4200
     3bc: 414c         	lw	a1, 0x4(a0)
     3be: c005f593     	andi	a1, a1, -0x400
     3c2: 0585         	addi	a1, a1, 0x1
     3c4: c14c         	sw	a1, 0x4(a0)
     3c6: 13c12083     	lw	ra, 0x13c(sp)
     3ca: 13812283     	lw	t0, 0x138(sp)
     3ce: 13412303     	lw	t1, 0x134(sp)
     3d2: 13012383     	lw	t2, 0x130(sp)
     3d6: 12c12503     	lw	a0, 0x12c(sp)
     3da: 12812583     	lw	a1, 0x128(sp)
     3de: 12412603     	lw	a2, 0x124(sp)
     3e2: 12012683     	lw	a3, 0x120(sp)
     3e6: 11c12703     	lw	a4, 0x11c(sp)
     3ea: 11812783     	lw	a5, 0x118(sp)
     3ee: 11412803     	lw	a6, 0x114(sp)
     3f2: 11012883     	lw	a7, 0x110(sp)
     3f6: 10c12e03     	lw	t3, 0x10c(sp)
     3fa: 10812e83     	lw	t4, 0x108(sp)
     3fe: 10412f03     	lw	t5, 0x104(sp)
     402: 10012f83     	lw	t6, 0x100(sp)
     406: 306e         	fld	ft0, 0xf8(sp)
     408: 30ce         	fld	ft1, 0xf0(sp)
     40a: 312e         	fld	ft2, 0xe8(sp)
     40c: 318e         	fld	ft3, 0xe0(sp)
     40e: 226e         	fld	ft4, 0xd8(sp)
     410: 22ce         	fld	ft5, 0xd0(sp)
     412: 232e         	fld	ft6, 0xc8(sp)
     414: 238e         	fld	ft7, 0xc0(sp)
     416: 346a         	fld	fs0, 0xb8(sp)
     418: 34ca         	fld	fs1, 0xb0(sp)
     41a: 352a         	fld	fa0, 0xa8(sp)
     41c: 358a         	fld	fa1, 0xa0(sp)
     41e: 266a         	fld	fa2, 0x98(sp)
     420: 26ca         	fld	fa3, 0x90(sp)
     422: 272a         	fld	fa4, 0x88(sp)
     424: 278a         	fld	fa5, 0x80(sp)
     426: 3866         	fld	fa6, 0x78(sp)
     428: 38c6         	fld	fa7, 0x70(sp)
     42a: 3926         	fld	fs2, 0x68(sp)
     42c: 3986         	fld	fs3, 0x60(sp)
     42e: 2a66         	fld	fs4, 0x58(sp)
     430: 2ac6         	fld	fs5, 0x50(sp)
     432: 2b26         	fld	fs6, 0x48(sp)
     434: 2b86         	fld	fs7, 0x40(sp)
     436: 3c62         	fld	fs8, 0x38(sp)
     438: 3cc2         	fld	fs9, 0x30(sp)
     43a: 3d22         	fld	fs10, 0x28(sp)
     43c: 3d82         	fld	fs11, 0x20(sp)
     43e: 2e62         	fld	ft8, 0x18(sp)
     440: 2ec2         	fld	ft9, 0x10(sp)
     442: 2f22         	fld	ft10, 0x8(sp)
     444: 2f82         	fld	ft11, 0x0(sp)
     446: 6131         	addi	sp, sp, 0x140
     448: 30200073     	mret

0000044c <GPIO0_B>:
     44c: 7129         	addi	sp, sp, -0x140
     44e: 12112e23     	sw	ra, 0x13c(sp)
     452: 12512c23     	sw	t0, 0x138(sp)
     456: 12612a23     	sw	t1, 0x134(sp)
     45a: 12712823     	sw	t2, 0x130(sp)
     45e: 12a12623     	sw	a0, 0x12c(sp)
     462: 0208         	addi	a0, sp, 0x100
     464: d50c         	sw	a1, 0x28(a0)
     466: d150         	sw	a2, 0x24(a0)
     468: d114         	sw	a3, 0x20(a0)
     46a: cd58         	sw	a4, 0x1c(a0)
     46c: cd1c         	sw	a5, 0x18(a0)
     46e: 01052a23     	sw	a6, 0x14(a0)
     472: 01152823     	sw	a7, 0x10(a0)
     476: 01c52623     	sw	t3, 0xc(a0)
     47a: 01d52423     	sw	t4, 0x8(a0)
     47e: 01e52223     	sw	t5, 0x4(a0)
     482: 01f52023     	sw	t6, 0x0(a0)
     486: bd82         	fsd	ft0, 0xf8(sp)
     488: b986         	fsd	ft1, 0xf0(sp)
     48a: b58a         	fsd	ft2, 0xe8(sp)
     48c: b18e         	fsd	ft3, 0xe0(sp)
     48e: ad92         	fsd	ft4, 0xd8(sp)
     490: a996         	fsd	ft5, 0xd0(sp)
     492: a59a         	fsd	ft6, 0xc8(sp)
     494: a19e         	fsd	ft7, 0xc0(sp)
     496: bd22         	fsd	fs0, 0xb8(sp)
     498: b926         	fsd	fs1, 0xb0(sp)
     49a: b52a         	fsd	fa0, 0xa8(sp)
     49c: b12e         	fsd	fa1, 0xa0(sp)
     49e: ad32         	fsd	fa2, 0x98(sp)
     4a0: a936         	fsd	fa3, 0x90(sp)
     4a2: a53a         	fsd	fa4, 0x88(sp)
     4a4: a13e         	fsd	fa5, 0x80(sp)
     4a6: bcc2         	fsd	fa6, 0x78(sp)
     4a8: b8c6         	fsd	fa7, 0x70(sp)
     4aa: b4ca         	fsd	fs2, 0x68(sp)
     4ac: b0ce         	fsd	fs3, 0x60(sp)
     4ae: acd2         	fsd	fs4, 0x58(sp)
     4b0: a8d6         	fsd	fs5, 0x50(sp)
     4b2: a4da         	fsd	fs6, 0x48(sp)
     4b4: a0de         	fsd	fs7, 0x40(sp)
     4b6: bc62         	fsd	fs8, 0x38(sp)
     4b8: b866         	fsd	fs9, 0x30(sp)
     4ba: b46a         	fsd	fs10, 0x28(sp)
     4bc: b06e         	fsd	fs11, 0x20(sp)
     4be: ac72         	fsd	ft8, 0x18(sp)
     4c0: a876         	fsd	ft9, 0x10(sp)
     4c2: a47a         	fsd	ft10, 0x8(sp)
     4c4: a07e         	fsd	ft11, 0x0(sp)
     4c6: 4505         	li	a0, 0x1
     4c8: 80003097     	auipc	ra, 0x80003
     4cc: 760080e7     	jalr	0x760(ra) <hpm_hal::gpio::input_future::on_interrupt::hcc22a711960b534d>
     4d0: e4200537     	lui	a0, 0xe4200
     4d4: 414c         	lw	a1, 0x4(a0)
     4d6: c005f593     	andi	a1, a1, -0x400
     4da: 0589         	addi	a1, a1, 0x2
     4dc: c14c         	sw	a1, 0x4(a0)
     4de: 13c12083     	lw	ra, 0x13c(sp)
     4e2: 13812283     	lw	t0, 0x138(sp)
     4e6: 13412303     	lw	t1, 0x134(sp)
     4ea: 13012383     	lw	t2, 0x130(sp)
     4ee: 12c12503     	lw	a0, 0x12c(sp)
     4f2: 12812583     	lw	a1, 0x128(sp)
     4f6: 12412603     	lw	a2, 0x124(sp)
     4fa: 12012683     	lw	a3, 0x120(sp)
     4fe: 11c12703     	lw	a4, 0x11c(sp)
     502: 11812783     	lw	a5, 0x118(sp)
     506: 11412803     	lw	a6, 0x114(sp)
     50a: 11012883     	lw	a7, 0x110(sp)
     50e: 10c12e03     	lw	t3, 0x10c(sp)
     512: 10812e83     	lw	t4, 0x108(sp)
     516: 10412f03     	lw	t5, 0x104(sp)
     51a: 10012f83     	lw	t6, 0x100(sp)
     51e: 306e         	fld	ft0, 0xf8(sp)
     520: 30ce         	fld	ft1, 0xf0(sp)
     522: 312e         	fld	ft2, 0xe8(sp)
     524: 318e         	fld	ft3, 0xe0(sp)
     526: 226e         	fld	ft4, 0xd8(sp)
     528: 22ce         	fld	ft5, 0xd0(sp)
     52a: 232e         	fld	ft6, 0xc8(sp)
     52c: 238e         	fld	ft7, 0xc0(sp)
     52e: 346a         	fld	fs0, 0xb8(sp)
     530: 34ca         	fld	fs1, 0xb0(sp)
     532: 352a         	fld	fa0, 0xa8(sp)
     534: 358a         	fld	fa1, 0xa0(sp)
     536: 266a         	fld	fa2, 0x98(sp)
     538: 26ca         	fld	fa3, 0x90(sp)
     53a: 272a         	fld	fa4, 0x88(sp)
     53c: 278a         	fld	fa5, 0x80(sp)
     53e: 3866         	fld	fa6, 0x78(sp)
     540: 38c6         	fld	fa7, 0x70(sp)
     542: 3926         	fld	fs2, 0x68(sp)
     544: 3986         	fld	fs3, 0x60(sp)
     546: 2a66         	fld	fs4, 0x58(sp)
     548: 2ac6         	fld	fs5, 0x50(sp)
     54a: 2b26         	fld	fs6, 0x48(sp)
     54c: 2b86         	fld	fs7, 0x40(sp)
     54e: 3c62         	fld	fs8, 0x38(sp)
     550: 3cc2         	fld	fs9, 0x30(sp)
     552: 3d22         	fld	fs10, 0x28(sp)
     554: 3d82         	fld	fs11, 0x20(sp)
     556: 2e62         	fld	ft8, 0x18(sp)
     558: 2ec2         	fld	ft9, 0x10(sp)
     55a: 2f22         	fld	ft10, 0x8(sp)
     55c: 2f82         	fld	ft11, 0x0(sp)
     55e: 6131         	addi	sp, sp, 0x140
     560: 30200073     	mret

00000564 <GPIO0_Y>:
     564: 7129         	addi	sp, sp, -0x140
     566: 12112e23     	sw	ra, 0x13c(sp)
     56a: 12512c23     	sw	t0, 0x138(sp)
     56e: 12612a23     	sw	t1, 0x134(sp)
     572: 12712823     	sw	t2, 0x130(sp)
     576: 12a12623     	sw	a0, 0x12c(sp)
     57a: 0208         	addi	a0, sp, 0x100
     57c: d50c         	sw	a1, 0x28(a0)
     57e: d150         	sw	a2, 0x24(a0)
     580: d114         	sw	a3, 0x20(a0)
     582: cd58         	sw	a4, 0x1c(a0)
     584: cd1c         	sw	a5, 0x18(a0)
     586: 01052a23     	sw	a6, 0x14(a0)
     58a: 01152823     	sw	a7, 0x10(a0)
     58e: 01c52623     	sw	t3, 0xc(a0)
     592: 01d52423     	sw	t4, 0x8(a0)
     596: 01e52223     	sw	t5, 0x4(a0)
     59a: 01f52023     	sw	t6, 0x0(a0)
     59e: bd82         	fsd	ft0, 0xf8(sp)
     5a0: b986         	fsd	ft1, 0xf0(sp)
     5a2: b58a         	fsd	ft2, 0xe8(sp)
     5a4: b18e         	fsd	ft3, 0xe0(sp)
     5a6: ad92         	fsd	ft4, 0xd8(sp)
     5a8: a996         	fsd	ft5, 0xd0(sp)
     5aa: a59a         	fsd	ft6, 0xc8(sp)
     5ac: a19e         	fsd	ft7, 0xc0(sp)
     5ae: bd22         	fsd	fs0, 0xb8(sp)
     5b0: b926         	fsd	fs1, 0xb0(sp)
     5b2: b52a         	fsd	fa0, 0xa8(sp)
     5b4: b12e         	fsd	fa1, 0xa0(sp)
     5b6: ad32         	fsd	fa2, 0x98(sp)
     5b8: a936         	fsd	fa3, 0x90(sp)
     5ba: a53a         	fsd	fa4, 0x88(sp)
     5bc: a13e         	fsd	fa5, 0x80(sp)
     5be: bcc2         	fsd	fa6, 0x78(sp)
     5c0: b8c6         	fsd	fa7, 0x70(sp)
     5c2: b4ca         	fsd	fs2, 0x68(sp)
     5c4: b0ce         	fsd	fs3, 0x60(sp)
     5c6: acd2         	fsd	fs4, 0x58(sp)
     5c8: a8d6         	fsd	fs5, 0x50(sp)
     5ca: a4da         	fsd	fs6, 0x48(sp)
     5cc: a0de         	fsd	fs7, 0x40(sp)
     5ce: bc62         	fsd	fs8, 0x38(sp)
     5d0: b866         	fsd	fs9, 0x30(sp)
     5d2: b46a         	fsd	fs10, 0x28(sp)
     5d4: b06e         	fsd	fs11, 0x20(sp)
     5d6: ac72         	fsd	ft8, 0x18(sp)
     5d8: a876         	fsd	ft9, 0x10(sp)
     5da: a47a         	fsd	ft10, 0x8(sp)
     5dc: a07e         	fsd	ft11, 0x0(sp)
     5de: 4539         	li	a0, 0xe
     5e0: 80003097     	auipc	ra, 0x80003
     5e4: 648080e7     	jalr	0x648(ra) <hpm_hal::gpio::input_future::on_interrupt::hcc22a711960b534d>
     5e8: e4200537     	lui	a0, 0xe4200
     5ec: 414c         	lw	a1, 0x4(a0)
     5ee: c005f593     	andi	a1, a1, -0x400
     5f2: 0591         	addi	a1, a1, 0x4
     5f4: c14c         	sw	a1, 0x4(a0)
     5f6: 13c12083     	lw	ra, 0x13c(sp)
     5fa: 13812283     	lw	t0, 0x138(sp)
     5fe: 13412303     	lw	t1, 0x134(sp)
     602: 13012383     	lw	t2, 0x130(sp)
     606: 12c12503     	lw	a0, 0x12c(sp)
     60a: 12812583     	lw	a1, 0x128(sp)
     60e: 12412603     	lw	a2, 0x124(sp)
     612: 12012683     	lw	a3, 0x120(sp)
     616: 11c12703     	lw	a4, 0x11c(sp)
     61a: 11812783     	lw	a5, 0x118(sp)
     61e: 11412803     	lw	a6, 0x114(sp)
     622: 11012883     	lw	a7, 0x110(sp)
     626: 10c12e03     	lw	t3, 0x10c(sp)
     62a: 10812e83     	lw	t4, 0x108(sp)
     62e: 10412f03     	lw	t5, 0x104(sp)
     632: 10012f83     	lw	t6, 0x100(sp)
     636: 306e         	fld	ft0, 0xf8(sp)
     638: 30ce         	fld	ft1, 0xf0(sp)
     63a: 312e         	fld	ft2, 0xe8(sp)
     63c: 318e         	fld	ft3, 0xe0(sp)
     63e: 226e         	fld	ft4, 0xd8(sp)
     640: 22ce         	fld	ft5, 0xd0(sp)
     642: 232e         	fld	ft6, 0xc8(sp)
     644: 238e         	fld	ft7, 0xc0(sp)
     646: 346a         	fld	fs0, 0xb8(sp)
     648: 34ca         	fld	fs1, 0xb0(sp)
     64a: 352a         	fld	fa0, 0xa8(sp)
     64c: 358a         	fld	fa1, 0xa0(sp)
     64e: 266a         	fld	fa2, 0x98(sp)
     650: 26ca         	fld	fa3, 0x90(sp)
     652: 272a         	fld	fa4, 0x88(sp)
     654: 278a         	fld	fa5, 0x80(sp)
     656: 3866         	fld	fa6, 0x78(sp)
     658: 38c6         	fld	fa7, 0x70(sp)
     65a: 3926         	fld	fs2, 0x68(sp)
     65c: 3986         	fld	fs3, 0x60(sp)
     65e: 2a66         	fld	fs4, 0x58(sp)
     660: 2ac6         	fld	fs5, 0x50(sp)
     662: 2b26         	fld	fs6, 0x48(sp)
     664: 2b86         	fld	fs7, 0x40(sp)
     666: 3c62         	fld	fs8, 0x38(sp)
     668: 3cc2         	fld	fs9, 0x30(sp)
     66a: 3d22         	fld	fs10, 0x28(sp)
     66c: 3d82         	fld	fs11, 0x20(sp)
     66e: 2e62         	fld	ft8, 0x18(sp)
     670: 2ec2         	fld	ft9, 0x10(sp)
     672: 2f22         	fld	ft10, 0x8(sp)
     674: 2f82         	fld	ft11, 0x0(sp)
     676: 6131         	addi	sp, sp, 0x140
     678: 30200073     	mret

0000067c <_start_rust_CORE_LOCAL>:
     67c: 1141         	addi	sp, sp, -0x10
     67e: c606         	sw	ra, 0xc(sp)
     680: c422         	sw	s0, 0x8(sp)
     682: 342025f3     	csrr	a1, mcause
     686: 842a         	mv	s0, a0
     688: 49f59513     	bclri	a0, a1, 0x1f
     68c: 0205cd63     	bltz	a1, 0x6c6 <.Lline_table_start16+0x24>
     690: 45c1         	li	a1, 0x10
     692: 00b57d63     	bgeu	a0, a1, 0x6ac <.Lline_table_start16+0xa>
     696: 800045b7     	lui	a1, 0x80004
     69a: 27058593     	addi	a1, a1, 0x270
     69e: 20b54533     	sh2add	a0, a0, a1
     6a2: 410c         	lw	a1, 0x0(a0)
     6a4: c581         	beqz	a1, 0x6ac <.Lline_table_start16+0xa>
     6a6: 8522         	mv	a0, s0
     6a8: 9582         	jalr	a1
     6aa: a031         	j	0x6b6 <.Lline_table_start16+0x14>
     6ac: 8522         	mv	a0, s0
     6ae: 80003097     	auipc	ra, 0x80003
     6b2: 612080e7     	jalr	0x612(ra) <UserEnvCall>
     6b6: 8522         	mv	a0, s0
     6b8: 40b2         	lw	ra, 0xc(sp)
     6ba: 4422         	lw	s0, 0x8(sp)
     6bc: 0141         	addi	sp, sp, 0x10
     6be: 80003317     	auipc	t1, 0x80003
     6c2: 60230067     	jr	0x602(t1) <UserEnvCall>
     6c6: 45b9         	li	a1, 0xe
     6c8: 00b57e63     	bgeu	a0, a1, 0x6e4 <.Lline_table_start16+0x42>
     6cc: 800045b7     	lui	a1, 0x80004
     6d0: 2b058593     	addi	a1, a1, 0x2b0
     6d4: 20b54533     	sh2add	a0, a0, a1
     6d8: 411c         	lw	a5, 0x0(a0)
     6da: c789         	beqz	a5, 0x6e4 <.Lline_table_start16+0x42>
     6dc: 40b2         	lw	ra, 0xc(sp)
     6de: 4422         	lw	s0, 0x8(sp)
     6e0: 0141         	addi	sp, sp, 0x10
     6e2: 8782         	jr	a5
     6e4: 40b2         	lw	ra, 0xc(sp)
     6e6: 4422         	lw	s0, 0x8(sp)
     6e8: 0141         	addi	sp, sp, 0x10
     6ea: 80003317     	auipc	t1, 0x80003
     6ee: 5d830067     	jr	0x5d8(t1) <XPI0>
     6f2: 0000         	unimp
     6f4: 0000         	unimp
     6f6: 0000         	unimp

Disassembly of section .text:

80003748 <defmt::export::fmt::h887e4e68a4bb6559>:
80003748: 1141         	addi	sp, sp, -0x10
8000374a: c606         	sw	ra, 0xc(sp)
8000374c: c422         	sw	s0, 0x8(sp)
8000374e: 842a         	mv	s0, a0
80003750: 00000537     	lui	a0, 0x0
80003754: 00350513     	addi	a0, a0, 0x3
80003758: 00000097     	auipc	ra, 0x0
8000375c: 162080e7     	jalr	0x162(ra) <defmt::export::istr::h093ac2a86638ac0a>
80003760: c222         	sw	s0, 0x4(sp)
80003762: 0048         	addi	a0, sp, 0x4
80003764: 4591         	li	a1, 0x4
80003766: 00000097     	auipc	ra, 0x0
8000376a: 38c080e7     	jalr	0x38c(ra) <_defmt_write>
8000376e: 40b2         	lw	ra, 0xc(sp)
80003770: 4422         	lw	s0, 0x8(sp)
80003772: 0141         	addi	sp, sp, 0x10
80003774: 8082         	ret

80003776 <rust_begin_unwind>:
80003776: 1141         	addi	sp, sp, -0x10
80003778: c606         	sw	ra, 0xc(sp)
8000377a: 00000097     	auipc	ra, 0x0
8000377e: 2b2080e7     	jalr	0x2b2(ra) <_defmt_acquire>
80003782: 00000537     	lui	a0, 0x0
80003786: 00750513     	addi	a0, a0, 0x7
8000378a: 00000097     	auipc	ra, 0x0
8000378e: 14c080e7     	jalr	0x14c(ra) <defmt::export::header::h80e6f83949247917>
80003792: 00000097     	auipc	ra, 0x0
80003796: 2ea080e7     	jalr	0x2ea(ra) <_defmt_release>
8000379a: a001         	j	0x8000379a <rust_begin_unwind+0x24>

8000379c <main>:
8000379c: 1141         	addi	sp, sp, -0x10
8000379e: c606         	sw	ra, 0xc(sp)
800037a0: c422         	sw	s0, 0x8(sp)
800037a2: c226         	sw	s1, 0x4(sp)
800037a4: 6519         	lui	a0, 0x6
800037a6: cb250513     	addi	a0, a0, -0x34e
800037aa: 00a11023     	sh	a0, 0x0(sp)
800037ae: 00000097     	auipc	ra, 0x0
800037b2: 27e080e7     	jalr	0x27e(ra) <_defmt_acquire>
800037b6: 00000537     	lui	a0, 0x0
800037ba: 00450513     	addi	a0, a0, 0x4
800037be: 00000097     	auipc	ra, 0x0
800037c2: 118080e7     	jalr	0x118(ra) <defmt::export::header::h80e6f83949247917>
800037c6: 00000537     	lui	a0, 0x0
800037ca: 00150513     	addi	a0, a0, 0x1
800037ce: 00000097     	auipc	ra, 0x0
800037d2: 0ec080e7     	jalr	0xec(ra) <defmt::export::istr::h093ac2a86638ac0a>
800037d6: 00000537     	lui	a0, 0x0
800037da: 00250513     	addi	a0, a0, 0x2
800037de: 00000097     	auipc	ra, 0x0
800037e2: 0dc080e7     	jalr	0xdc(ra) <defmt::export::istr::h093ac2a86638ac0a>
800037e6: 4409         	li	s0, 0x2
800037e8: 848a         	mv	s1, sp
800037ea: cc19         	beqz	s0, 0x80003808 <main+0x6c>
800037ec: 0004c503     	lbu	a0, 0x0(s1)
800037f0: 00a101a3     	sb	a0, 0x3(sp)
800037f4: 00310513     	addi	a0, sp, 0x3
800037f8: 4585         	li	a1, 0x1
800037fa: 00000097     	auipc	ra, 0x0
800037fe: 2f8080e7     	jalr	0x2f8(ra) <_defmt_write>
80003802: 147d         	addi	s0, s0, -0x1
80003804: 0485         	addi	s1, s1, 0x1
80003806: f07d         	bnez	s0, 0x800037ec <main+0x50>
80003808: 00000097     	auipc	ra, 0x0
8000380c: 274080e7     	jalr	0x274(ra) <_defmt_release>
80003810: 0001         	nop
80003812: 0001         	nop
80003814: 0001         	nop
80003816: f1f2f537     	lui	a0, 0xf1f2f
8000381a: 3f450513     	addi	a0, a0, 0x3f4
8000381e: f5f6f5b7     	lui	a1, 0xf5f6f
80003822: 7f858593     	addi	a1, a1, 0x7f8
80003826: 047f 48b5 0097 0000 80e7     	<unknown>
80003830: 2020         	fld	fs0, 0x40(s0)
80003832: 00000537     	lui	a0, 0x0
80003836: 00550513     	addi	a0, a0, 0x5
8000383a: 00000097     	auipc	ra, 0x0
8000383e: 09c080e7     	jalr	0x9c(ra) <defmt::export::header::h80e6f83949247917>
80003842: 8522         	mv	a0, s0
80003844: 00000097     	auipc	ra, 0x0
80003848: f04080e7     	jalr	-0xfc(ra) <defmt::export::fmt::h887e4e68a4bb6559>
8000384c: 00000097     	auipc	ra, 0x0
80003850: 230080e7     	jalr	0x230(ra) <_defmt_release>
80003854: 00000097     	auipc	ra, 0x0
80003858: 1d8080e7     	jalr	0x1d8(ra) <_defmt_acquire>
8000385c: 00000537     	lui	a0, 0x0
80003860: 00650513     	addi	a0, a0, 0x6
80003864: 00000097     	auipc	ra, 0x0
80003868: 072080e7     	jalr	0x72(ra) <defmt::export::header::h80e6f83949247917>
8000386c: e7e9f537     	lui	a0, 0xe7e9f
80003870: bec50513     	addi	a0, a0, -0x414
80003874: 00000097     	auipc	ra, 0x0
80003878: ed4080e7     	jalr	-0x12c(ra) <defmt::export::fmt::h887e4e68a4bb6559>
8000387c: 00000097     	auipc	ra, 0x0
80003880: 200080e7     	jalr	0x200(ra) <_defmt_release>
80003884: 10500073     	wfi
80003888: bff5         	j	0x80003884 <main+0xe8>

8000388a <core::panicking::panic_fmt::h525aebbf60a29a7a>:
8000388a: 00000097     	auipc	ra, 0x0
8000388e: eec080e7     	jalr	-0x114(ra) <rust_begin_unwind>

80003892 <core::panicking::panic_explicit::he35d97b6f5738f97>:
80003892: 00000097     	auipc	ra, 0x0
80003896: 008080e7     	jalr	0x8(ra) <core::panicking::panic_display::hb19295cbc2adba88>

8000389a <core::panicking::panic_display::hb19295cbc2adba88>:
8000389a: 00000097     	auipc	ra, 0x0
8000389e: ff0080e7     	jalr	-0x10(ra) <core::panicking::panic_fmt::h525aebbf60a29a7a>

800038a2 <core::slice::index::slice_start_index_len_fail::hd4f0ab31ae985065>:
800038a2: 00000097     	auipc	ra, 0x0
800038a6: 008080e7     	jalr	0x8(ra) <core::slice::index::slice_start_index_len_fail_rt::hc95c25ab6b32d1a9>

800038aa <core::slice::index::slice_start_index_len_fail_rt::hc95c25ab6b32d1a9>:
800038aa: 00000097     	auipc	ra, 0x0
800038ae: fe0080e7     	jalr	-0x20(ra) <core::panicking::panic_fmt::h525aebbf60a29a7a>

800038b2 <core::panicking::panic_const::panic_const_div_by_zero::h4fc2a9d0290dad64>:
800038b2: 00000097     	auipc	ra, 0x0
800038b6: fd8080e7     	jalr	-0x28(ra) <core::panicking::panic_fmt::h525aebbf60a29a7a>

800038ba <defmt::export::istr::h093ac2a86638ac0a>:
800038ba: 1141         	addi	sp, sp, -0x10
800038bc: c606         	sw	ra, 0xc(sp)
800038be: 00a11523     	sh	a0, 0xa(sp)
800038c2: 00a10513     	addi	a0, sp, 0xa
800038c6: 4589         	li	a1, 0x2
800038c8: 00000097     	auipc	ra, 0x0
800038cc: 22a080e7     	jalr	0x22a(ra) <_defmt_write>
800038d0: 40b2         	lw	ra, 0xc(sp)
800038d2: 0141         	addi	sp, sp, 0x10
800038d4: 8082         	ret

800038d6 <defmt::export::header::h80e6f83949247917>:
800038d6: 1141         	addi	sp, sp, -0x10
800038d8: c606         	sw	ra, 0xc(sp)
800038da: 00000097     	auipc	ra, 0x0
800038de: fe0080e7     	jalr	-0x20(ra) <defmt::export::istr::h093ac2a86638ac0a>
800038e2: 40b2         	lw	ra, 0xc(sp)
800038e4: 0141         	addi	sp, sp, 0x10
800038e6: 00000317     	auipc	t1, 0x0
800038ea: 3b830067     	jr	0x3b8(t1) <_defmt_timestamp>

800038ee <__defmt_default_timestamp>:
800038ee: 8082         	ret

800038f0 <__defmt_default_panic>:
800038f0: 00000097     	auipc	ra, 0x0
800038f4: 008080e7     	jalr	0x8(ra) <defmt::default_panic::panic_cold_explicit::ha53590dffbae7ec1>

800038f8 <defmt::default_panic::panic_cold_explicit::ha53590dffbae7ec1>:
800038f8: 00000097     	auipc	ra, 0x0
800038fc: f9a080e7     	jalr	-0x66(ra) <core::panicking::panic_explicit::he35d97b6f5738f97>

80003900 <core::ops::function::FnMut::call_mut::h4ad2e262bfecd66c>:
80003900: 00000297     	auipc	t0, 0x0
80003904: 5f6282e7     	jalr	t0, 0x5f6(t0) <OUTLINED_FUNCTION_0>
80003908: 000804b7     	lui	s1, 0x80
8000390c: 00048493     	mv	s1, s1
80003910: 54cc         	lw	a1, 0x2c(s1)
80003912: 898d         	andi	a1, a1, 0x3
80003914: 4609         	li	a2, 0x2
80003916: 842a         	mv	s0, a0
80003918: 00c58763     	beq	a1, a2, 0x80003926 <core::ops::function::FnMut::call_mut::h4ad2e262bfecd66c+0x26>
8000391c: 80004937     	lui	s2, 0x80004
80003920: 99690913     	addi	s2, s2, -0x66a
80003924: a029         	j	0x8000392e <core::ops::function::FnMut::call_mut::h4ad2e262bfecd66c+0x2e>
80003926: 80004937     	lui	s2, 0x80004
8000392a: 96490913     	addi	s2, s2, -0x69c
8000392e: 4501         	li	a0, 0x0
80003930: 04e1         	addi	s1, s1, 0x18
80003932: 4985         	li	s3, 0x1
80003934: 8905         	andi	a0, a0, 0x1
80003936: e919         	bnez	a0, 0x8000394c <core::ops::function::FnMut::call_mut::h4ad2e262bfecd66c+0x4c>
80003938: 4605         	li	a2, 0x1
8000393a: 8526         	mv	a0, s1
8000393c: 85a2         	mv	a1, s0
8000393e: 9902         	jalr	s2
80003940: dd65         	beqz	a0, 0x80003938 <core::ops::function::FnMut::call_mut::h4ad2e262bfecd66c+0x38>
80003942: 01351c63     	bne	a0, s3, 0x8000395a <core::ops::function::FnMut::call_mut::h4ad2e262bfecd66c+0x5a>
80003946: 0405         	addi	s0, s0, 0x1
80003948: 4505         	li	a0, 0x1
8000394a: b7ed         	j	0x80003934 <core::ops::function::FnMut::call_mut::h4ad2e262bfecd66c+0x34>
8000394c: 40f2         	lw	ra, 0x1c(sp)
8000394e: 4462         	lw	s0, 0x18(sp)
80003950: 44d2         	lw	s1, 0x14(sp)
80003952: 4942         	lw	s2, 0x10(sp)
80003954: 49b2         	lw	s3, 0xc(sp)
80003956: 6105         	addi	sp, sp, 0x20
80003958: 8082         	ret
8000395a: 4585         	li	a1, 0x1
8000395c: 00000097     	auipc	ra, 0x0
80003960: f46080e7     	jalr	-0xba(ra) <core::slice::index::slice_start_index_len_fail::hd4f0ab31ae985065>

80003964 <defmt_rtt::channel::Channel::blocking_write::h86389ed551ff02e3>:
80003964: c61d         	beqz	a2, 0x80003992 <defmt_rtt::channel::Channel::blocking_write::h86389ed551ff02e3+0x2e>
80003966: 4918         	lw	a4, 0x10(a0)
80003968: 4554         	lw	a3, 0xc(a0)
8000396a: 0230000f     	fence	r, rw
8000396e: 00e6fa63     	bgeu	a3, a4, 0x80003982 <defmt_rtt::channel::Channel::blocking_write::h86389ed551ff02e3+0x1e>
80003972: fff6c793     	not	a5, a3
80003976: 973e         	add	a4, a4, a5
80003978: cf09         	beqz	a4, 0x80003992 <defmt_rtt::channel::Channel::blocking_write::h86389ed551ff02e3+0x2e>
8000397a: 00000317     	auipc	t1, 0x0
8000397e: 02e30067     	jr	0x2e(t1) <defmt_rtt::channel::Channel::write_impl::h2421741f32af634d>
80003982: c701         	beqz	a4, 0x8000398a <defmt_rtt::channel::Channel::blocking_write::h86389ed551ff02e3+0x26>
80003984: 40000713     	li	a4, 0x400
80003988: a019         	j	0x8000398e <defmt_rtt::channel::Channel::blocking_write::h86389ed551ff02e3+0x2a>
8000398a: 3ff00713     	li	a4, 0x3ff
8000398e: 8f15         	sub	a4, a4, a3
80003990: f76d         	bnez	a4, 0x8000397a <defmt_rtt::channel::Channel::blocking_write::h86389ed551ff02e3+0x16>
80003992: 4501         	li	a0, 0x0
80003994: 8082         	ret

80003996 <defmt_rtt::channel::Channel::nonblocking_write::h43708078bbcfc5b7>:
80003996: 4554         	lw	a3, 0xc(a0)
80003998: 40000713     	li	a4, 0x400
8000399c: 0230000f     	fence	r, rw
800039a0: 00000317     	auipc	t1, 0x0
800039a4: 00830067     	jr	0x8(t1) <defmt_rtt::channel::Channel::write_impl::h2421741f32af634d>

800039a8 <defmt_rtt::channel::Channel::write_impl::h2421741f32af634d>:
800039a8: 00000297     	auipc	t0, 0x0
800039ac: 54e282e7     	jalr	t0, 0x54e(t0) <OUTLINED_FUNCTION_0>
800039b0: c452         	sw	s4, 0x8(sp)
800039b2: 0ae65433     	minu	s0, a2, a4
800039b6: 00d40a33     	add	s4, s0, a3
800039ba: 40000613     	li	a2, 0x400
800039be: 892e         	mv	s2, a1
800039c0: 84aa         	mv	s1, a0
800039c2: 03467263     	bgeu	a2, s4, 0x800039e6 <defmt_rtt::channel::Channel::write_impl::h2421741f32af634d+0x3e>
800039c6: 40c8         	lw	a0, 0x4(s1)
800039c8: 40d609b3     	sub	s3, a2, a3
800039cc: 9536         	add	a0, a0, a3
800039ce: 85ca         	mv	a1, s2
800039d0: 864e         	mv	a2, s3
800039d2: 00001097     	auipc	ra, 0x1
800039d6: 890080e7     	jalr	-0x770(ra) <memcpy>
800039da: 40c8         	lw	a0, 0x4(s1)
800039dc: 013905b3     	add	a1, s2, s3
800039e0: 41340633     	sub	a2, s0, s3
800039e4: a029         	j	0x800039ee <defmt_rtt::channel::Channel::write_impl::h2421741f32af634d+0x46>
800039e6: 40c8         	lw	a0, 0x4(s1)
800039e8: 9536         	add	a0, a0, a3
800039ea: 85ca         	mv	a1, s2
800039ec: 8622         	mv	a2, s0
800039ee: 00001097     	auipc	ra, 0x1
800039f2: 874080e7     	jalr	-0x78c(ra) <memcpy>
800039f6: 3ffa7513     	andi	a0, s4, 0x3ff
800039fa: 0310000f     	fence	rw, w
800039fe: c4c8         	sw	a0, 0xc(s1)
80003a00: 8522         	mv	a0, s0
80003a02: 40f2         	lw	ra, 0x1c(sp)
80003a04: 4462         	lw	s0, 0x18(sp)
80003a06: 44d2         	lw	s1, 0x14(sp)
80003a08: 4942         	lw	s2, 0x10(sp)
80003a0a: 49b2         	lw	s3, 0xc(sp)
80003a0c: 4a22         	lw	s4, 0x8(sp)
80003a0e: 6105         	addi	sp, sp, 0x20
80003a10: 8082         	ret

80003a12 <defmt::encoding::inner::Encoder::write::{{closure}}::h389ae927b364d1bb>:
80003a12: 1141         	addi	sp, sp, -0x10
80003a14: c606         	sw	ra, 0xc(sp)
80003a16: 00a105a3     	sb	a0, 0xb(sp)
80003a1a: 00b10513     	addi	a0, sp, 0xb
80003a1e: 00000097     	auipc	ra, 0x0
80003a22: ee2080e7     	jalr	-0x11e(ra) <core::ops::function::FnMut::call_mut::h4ad2e262bfecd66c>
80003a26: 40b2         	lw	ra, 0xc(sp)
80003a28: 0141         	addi	sp, sp, 0x10
80003a2a: 8082         	ret

80003a2c <_defmt_acquire>:
80003a2c: 1141         	addi	sp, sp, -0x10
80003a2e: c606         	sw	ra, 0xc(sp)
80003a30: 30047573     	csrrci	a0, mstatus, 0x8
80003a34: 00080637     	lui	a2, 0x80
80003a38: 45060583     	lb	a1, 0x450(a2)
80003a3c: ed85         	bnez	a1, 0x80003a74 <_defmt_acquire+0x48>
80003a3e: 4585         	li	a1, 0x1
80003a40: 44b60823     	sb	a1, 0x450(a2)
80003a44: 00080637     	lui	a2, 0x80
80003a48: 45264683     	lbu	a3, 0x452(a2)
80003a4c: 48355513     	bexti	a0, a0, 0x3
80003a50: 00080737     	lui	a4, 0x80
80003a54: 44a708a3     	sb	a0, 0x451(a4)
80003a58: ea99         	bnez	a3, 0x80003a6e <_defmt_acquire+0x42>
80003a5a: 44b60923     	sb	a1, 0x452(a2)
80003a5e: 000105a3     	sb	zero, 0xb(sp)
80003a62: 00b10513     	addi	a0, sp, 0xb
80003a66: 00000097     	auipc	ra, 0x0
80003a6a: e9a080e7     	jalr	-0x166(ra) <core::ops::function::FnMut::call_mut::h4ad2e262bfecd66c>
80003a6e: 40b2         	lw	ra, 0xc(sp)
80003a70: 0141         	addi	sp, sp, 0x10
80003a72: 8082         	ret
80003a74: 00000097     	auipc	ra, 0x0
80003a78: e16080e7     	jalr	-0x1ea(ra) <core::panicking::panic_fmt::h525aebbf60a29a7a>

80003a7c <_defmt_release>:
80003a7c: 1141         	addi	sp, sp, -0x10
80003a7e: c606         	sw	ra, 0xc(sp)
80003a80: c422         	sw	s0, 0x8(sp)
80003a82: 00080437     	lui	s0, 0x80
80003a86: 45344503     	lbu	a0, 0x453(s0)
80003a8a: c90d         	beqz	a0, 0x80003abc <_defmt_release+0x40>
80003a8c: 459d         	li	a1, 0x7
80003a8e: 00b57d63     	bgeu	a0, a1, 0x80003aa8 <_defmt_release+0x2c>
80003a92: 000805b7     	lui	a1, 0x80
80003a96: 4545c583     	lbu	a1, 0x454(a1)
80003a9a: 567d         	li	a2, -0x1
80003a9c: 00a61533     	sll	a0, a2, a0
80003aa0: 8d4d         	or	a0, a0, a1
80003aa2: 07f57513     	andi	a0, a0, 0x7f
80003aa6: a029         	j	0x80003ab0 <_defmt_release+0x34>
80003aa8: 07950513     	addi	a0, a0, 0x79
80003aac: f8056513     	ori	a0, a0, -0x80
80003ab0: 0ff57513     	andi	a0, a0, 0xff
80003ab4: 00000097     	auipc	ra, 0x0
80003ab8: f5e080e7     	jalr	-0xa2(ra) <defmt::encoding::inner::Encoder::write::{{closure}}::h389ae927b364d1bb>
80003abc: 4501         	li	a0, 0x0
80003abe: 00000097     	auipc	ra, 0x0
80003ac2: f54080e7     	jalr	-0xac(ra) <defmt::encoding::inner::Encoder::write::{{closure}}::h389ae927b364d1bb>
80003ac6: 440409a3     	sb	zero, 0x453(s0)
80003aca: 00080537     	lui	a0, 0x80
80003ace: 44050a23     	sb	zero, 0x454(a0)
80003ad2: 00080537     	lui	a0, 0x80
80003ad6: 44050823     	sb	zero, 0x450(a0)
80003ada: 00080537     	lui	a0, 0x80
80003ade: 45154503     	lbu	a0, 0x451(a0)
80003ae2: c501         	beqz	a0, 0x80003aea <_defmt_release+0x6e>
80003ae4: 4521         	li	a0, 0x8
80003ae6: 30052073     	csrs	mstatus, a0
80003aea: 40b2         	lw	ra, 0xc(sp)
80003aec: 4422         	lw	s0, 0x8(sp)
80003aee: 0141         	addi	sp, sp, 0x10
80003af0: 8082         	ret

80003af2 <_defmt_write>:
80003af2: 00000297     	auipc	t0, 0x0
80003af6: 404282e7     	jalr	t0, 0x404(t0) <OUTLINED_FUNCTION_0>
80003afa: c452         	sw	s4, 0x8(sp)
80003afc: c256         	sw	s5, 0x4(sp)
80003afe: c05a         	sw	s6, 0x0(sp)
80003b00: 000809b7     	lui	s3, 0x80
80003b04: 4539c483     	lbu	s1, 0x453(s3)
80003b08: 00080937     	lui	s2, 0x80
80003b0c: 45494a83     	lbu	s5, 0x454(s2)
80003b10: 842a         	mv	s0, a0
80003b12: 00b50b33     	add	s6, a0, a1
80003b16: 4a1d         	li	s4, 0x7
80003b18: 09640063     	beq	s0, s6, 0x80003b98 <_defmt_write+0xa6>
80003b1c: 00044503     	lbu	a0, 0x0(s0)
80003b20: 0ff4f593     	andi	a1, s1, 0xff
80003b24: 0345e363     	bltu	a1, s4, 0x80003b4a <_defmt_write+0x58>
80003b28: 0405         	addi	s0, s0, 0x1
80003b2a: c515         	beqz	a0, 0x80003b56 <_defmt_write+0x64>
80003b2c: 0485         	addi	s1, s1, 0x1
80003b2e: 00000097     	auipc	ra, 0x0
80003b32: ee4080e7     	jalr	-0x11c(ra) <defmt::encoding::inner::Encoder::write::{{closure}}::h389ae927b364d1bb>
80003b36: 0ff4f513     	andi	a0, s1, 0xff
80003b3a: f7a50513     	addi	a0, a0, -0x86
80003b3e: 449989a3     	sb	s1, 0x453(s3)
80003b42: f979         	bnez	a0, 0x80003b18 <_defmt_write+0x26>
80003b44: 0ff00a93     	li	s5, 0xff
80003b48: a81d         	j	0x80003b7e <_defmt_write+0x8c>
80003b4a: c919         	beqz	a0, 0x80003b60 <_defmt_write+0x6e>
80003b4c: 00000097     	auipc	ra, 0x0
80003b50: ec6080e7     	jalr	-0x13a(ra) <defmt::encoding::inner::Encoder::write::{{closure}}::h389ae927b364d1bb>
80003b54: a811         	j	0x80003b68 <_defmt_write+0x76>
80003b56: 07948513     	addi	a0, s1, 0x79
80003b5a: f8056a93     	ori	s5, a0, -0x80
80003b5e: a005         	j	0x80003b7e <_defmt_write+0x8c>
80003b60: 289a9ab3     	bset	s5, s5, s1
80003b64: 45590a23     	sb	s5, 0x454(s2)
80003b68: 0405         	addi	s0, s0, 0x1
80003b6a: 0485         	addi	s1, s1, 0x1
80003b6c: 0ff4f513     	andi	a0, s1, 0xff
80003b70: 449989a3     	sb	s1, 0x453(s3)
80003b74: fb4512e3     	bne	a0, s4, 0x80003b18 <_defmt_write+0x26>
80003b78: 0ffaf513     	andi	a0, s5, 0xff
80003b7c: dd51         	beqz	a0, 0x80003b18 <_defmt_write+0x26>
80003b7e: 0ffaf513     	andi	a0, s5, 0xff
80003b82: 00000097     	auipc	ra, 0x0
80003b86: e90080e7     	jalr	-0x170(ra) <defmt::encoding::inner::Encoder::write::{{closure}}::h389ae927b364d1bb>
80003b8a: 4a81         	li	s5, 0x0
80003b8c: 4481         	li	s1, 0x0
80003b8e: 440989a3     	sb	zero, 0x453(s3)
80003b92: 44090a23     	sb	zero, 0x454(s2)
80003b96: b749         	j	0x80003b18 <_defmt_write+0x26>
80003b98: 40f2         	lw	ra, 0x1c(sp)
80003b9a: 4462         	lw	s0, 0x18(sp)
80003b9c: 44d2         	lw	s1, 0x14(sp)
80003b9e: 4942         	lw	s2, 0x10(sp)
80003ba0: 49b2         	lw	s3, 0xc(sp)
80003ba2: 4a22         	lw	s4, 0x8(sp)
80003ba4: 4a92         	lw	s5, 0x4(sp)
80003ba6: 4b02         	lw	s6, 0x0(sp)
80003ba8: 6105         	addi	sp, sp, 0x20
80003baa: 8082         	ret

80003bac <embassy_sync::waitqueue::atomic_waker::AtomicWaker::wake::h356b0cfad1e8b1a3>:
80003bac: 00000297     	auipc	t0, 0x0
80003bb0: 34a282e7     	jalr	t0, 0x34a(t0) <OUTLINED_FUNCTION_0>
80003bb4: 842a         	mv	s0, a0
80003bb6: 30047573     	csrrci	a0, mstatus, 0x8
80003bba: 4004         	lw	s1, 0x0(s0)
80003bbc: 00442983     	lw	s3, 0x4(s0)
80003bc0: 00857913     	andi	s2, a0, 0x8
80003bc4: 00042023     	sw	zero, 0x0(s0)
80003bc8: cc81         	beqz	s1, 0x80003be0 <embassy_sync::waitqueue::atomic_waker::AtomicWaker::wake::h356b0cfad1e8b1a3+0x34>
80003bca: 448c         	lw	a1, 0x8(s1)
80003bcc: 854e         	mv	a0, s3
80003bce: 9582         	jalr	a1
80003bd0: 400c         	lw	a1, 0x0(s0)
80003bd2: 4048         	lw	a0, 0x4(s0)
80003bd4: c004         	sw	s1, 0x0(s0)
80003bd6: 01342223     	sw	s3, 0x4(s0)
80003bda: c199         	beqz	a1, 0x80003be0 <embassy_sync::waitqueue::atomic_waker::AtomicWaker::wake::h356b0cfad1e8b1a3+0x34>
80003bdc: 45cc         	lw	a1, 0xc(a1)
80003bde: 9582         	jalr	a1
80003be0: 00090563     	beqz	s2, 0x80003bea <embassy_sync::waitqueue::atomic_waker::AtomicWaker::wake::h356b0cfad1e8b1a3+0x3e>
80003be4: 4521         	li	a0, 0x8
80003be6: 30052073     	csrs	mstatus, a0
80003bea: 40f2         	lw	ra, 0x1c(sp)
80003bec: 4462         	lw	s0, 0x18(sp)
80003bee: 44d2         	lw	s1, 0x14(sp)
80003bf0: 4942         	lw	s2, 0x10(sp)
80003bf2: 49b2         	lw	s3, 0xc(sp)
80003bf4: 6105         	addi	sp, sp, 0x20
80003bf6: 8082         	ret

80003bf8 <<hpm_hal::embassy::time_driver_impl::MachineTimerDriver as embassy_time_driver::Driver>::now::h84f2869bdfaeabca>:
80003bf8: 1141         	addi	sp, sp, -0x10
80003bfa: c606         	sw	ra, 0xc(sp)
80003bfc: e6000537     	lui	a0, 0xe6000
80003c00: 414c         	lw	a1, 0x4(a0)
80003c02: 4108         	lw	a0, 0x0(a0)
80003c04: 00080637     	lui	a2, 0x80
80003c08: 03860613     	addi	a2, a2, 0x38
80003c0c: 4a10         	lw	a2, 0x10(a2)
80003c0e: ca09         	beqz	a2, 0x80003c20 <<hpm_hal::embassy::time_driver_impl::MachineTimerDriver as embassy_time_driver::Driver>::now::h84f2869bdfaeabca+0x28>
80003c10: 4681         	li	a3, 0x0
80003c12: 00000097     	auipc	ra, 0x0
80003c16: 2f2080e7     	jalr	0x2f2(ra) <__udivdi3>
80003c1a: 40b2         	lw	ra, 0xc(sp)
80003c1c: 0141         	addi	sp, sp, 0x10
80003c1e: 8082         	ret
80003c20: 00000097     	auipc	ra, 0x0
80003c24: c92080e7     	jalr	-0x36e(ra) <core::panicking::panic_const::panic_const_div_by_zero::h4fc2a9d0290dad64>

80003c28 <hpm_hal::gpio::input_future::on_interrupt::hcc22a711960b534d>:
80003c28: 1101         	addi	sp, sp, -0x20
80003c2a: ce06         	sw	ra, 0x1c(sp)
80003c2c: cc22         	sw	s0, 0x18(sp)
80003c2e: ca26         	sw	s1, 0x14(sp)
80003c30: c84a         	sw	s2, 0x10(sp)
80003c32: 0512         	slli	a0, a0, 0x4
80003c34: f00d0437     	lui	s0, 0xf00d0
80003c38: 942a         	add	s0, s0, a0
80003c3a: 30042503     	lw	a0, 0x300(s0)
80003c3e: c62a         	sw	a0, 0xc(sp)
80003c40: 497d         	li	s2, 0x1f
80003c42: 000804b7     	lui	s1, 0x80
80003c46: 45848493     	addi	s1, s1, 0x458
80003c4a: 0068         	addi	a0, sp, 0xc
80003c4c: 00000097     	auipc	ra, 0x0
80003c50: 03c080e7     	jalr	0x3c(ra) <<hpm_hal::internal::BitIter as core::iter::traits::iterator::Iterator>::next::h52a2ccad76180163>
80003c54: c105         	beqz	a0, 0x80003c74 <hpm_hal::gpio::input_future::on_interrupt::hcc22a711960b534d+0x4c>
80003c56: 28b01533     	bset	a0, zero, a1
80003c5a: 30a42023     	sw	a0, 0x300(s0)
80003c5e: 40a42423     	sw	a0, 0x408(s0)
80003c62: 00b96f63     	bltu	s2, a1, 0x80003c80 <hpm_hal::gpio::input_future::on_interrupt::hcc22a711960b534d+0x58>
80003c66: 2095e533     	sh3add	a0, a1, s1
80003c6a: 00000097     	auipc	ra, 0x0
80003c6e: f42080e7     	jalr	-0xbe(ra) <embassy_sync::waitqueue::atomic_waker::AtomicWaker::wake::h356b0cfad1e8b1a3>
80003c72: bfe1         	j	0x80003c4a <hpm_hal::gpio::input_future::on_interrupt::hcc22a711960b534d+0x22>
80003c74: 40f2         	lw	ra, 0x1c(sp)
80003c76: 4462         	lw	s0, 0x18(sp)
80003c78: 44d2         	lw	s1, 0x14(sp)
80003c7a: 4942         	lw	s2, 0x10(sp)
80003c7c: 6105         	addi	sp, sp, 0x20
80003c7e: 8082         	ret
80003c80: 00000097     	auipc	ra, 0x0
80003c84: c32080e7     	jalr	-0x3ce(ra) <core::panicking::panic_const::panic_const_div_by_zero::h4fc2a9d0290dad64>

80003c88 <<hpm_hal::internal::BitIter as core::iter::traits::iterator::Iterator>::next::h52a2ccad76180163>:
80003c88: 4110         	lw	a2, 0x0(a0)
80003c8a: 60161593     	ctz	a1, a2
80003c8e: c611         	beqz	a2, 0x80003c9a <<hpm_hal::internal::BitIter as core::iter::traits::iterator::Iterator>::next::h52a2ccad76180163+0x12>
80003c90: 48b61633     	bclr	a2, a2, a1
80003c94: c110         	sw	a2, 0x0(a0)
80003c96: 4505         	li	a0, 0x1
80003c98: 8082         	ret
80003c9a: 4501         	li	a0, 0x0
80003c9c: 8082         	ret

80003c9e <_defmt_timestamp>:
80003c9e: 1141         	addi	sp, sp, -0x10
80003ca0: c606         	sw	ra, 0xc(sp)
80003ca2: 00000097     	auipc	ra, 0x0
80003ca6: f56080e7     	jalr	-0xaa(ra) <<hpm_hal::embassy::time_driver_impl::MachineTimerDriver as embassy_time_driver::Driver>::now::h84f2869bdfaeabca>
80003caa: c22e         	sw	a1, 0x4(sp)
80003cac: c02a         	sw	a0, 0x0(sp)
80003cae: 850a         	mv	a0, sp
80003cb0: 45a1         	li	a1, 0x8
80003cb2: 00000097     	auipc	ra, 0x0
80003cb6: e40080e7     	jalr	-0x1c0(ra) <_defmt_write>
80003cba: 40b2         	lw	ra, 0xc(sp)
80003cbc: 0141         	addi	sp, sp, 0x10
80003cbe: 8082         	ret

80003cc0 <UserEnvCall>:
80003cc0: a001         	j	0x80003cc0 <UserEnvCall>

80003cc2 <XPI0>:
80003cc2: a001         	j	0x80003cc2 <XPI0>

80003cc4 <_setup_interrupts>:
80003cc4: 1141         	addi	sp, sp, -0x10
80003cc6: 4581         	li	a1, 0x0
80003cc8: e4200537     	lui	a0, 0xe4200
80003ccc: 00052023     	sw	zero, 0x0(a0)
80003cd0: c0058613     	addi	a2, a1, -0x400
80003cd4: ca01         	beqz	a2, 0x80003ce4 <_setup_interrupts+0x20>
80003cd6: 4150         	lw	a2, 0x4(a0)
80003cd8: c0067613     	andi	a2, a2, -0x400
80003cdc: 962e         	add	a2, a2, a1
80003cde: 0585         	addi	a1, a1, 0x1
80003ce0: c150         	sw	a2, 0x4(a0)
80003ce2: b7fd         	j	0x80003cd0 <_setup_interrupts+0xc>
80003ce4: e4002537     	lui	a0, 0xe4002
80003ce8: 4591         	li	a1, 0x4
80003cea: c591         	beqz	a1, 0x80003cf6 <_setup_interrupts+0x32>
80003cec: 00052023     	sw	zero, 0x0(a0)
80003cf0: 0511         	addi	a0, a0, 0x4
80003cf2: 15fd         	addi	a1, a1, -0x1
80003cf4: fde5         	bnez	a1, 0x80003cec <_setup_interrupts+0x28>
80003cf6: 4505         	li	a0, 0x1
80003cf8: 30652073     	csrs	mcounteren, a0
80003cfc: 00000537     	lui	a0, 0x0
80003d00: 00050513     	mv	a0, a0
80003d04: 30551073     	csrw	mtvec, a0
80003d08: e4000537     	lui	a0, 0xe4000
80003d0c: 410c         	lw	a1, 0x0(a0)
80003d0e: 0025e593     	ori	a1, a1, 0x2
80003d12: c10c         	sw	a1, 0x0(a0)
80003d14: 7d002573     	csrr	a0, 0x7d0
80003d18: c62a         	sw	a0, 0xc(sp)
80003d1a: 00256513     	ori	a0, a0, 0x2
80003d1e: c42a         	sw	a0, 0x8(sp)
80003d20: 7d051073     	csrw	0x7d0, a0
80003d24: 4521         	li	a0, 0x8
80003d26: 30052073     	csrs	mstatus, a0
80003d2a: 4509         	li	a0, 0x2
80003d2c: 30052073     	csrs	mstatus, a0
80003d30: 28b01513     	bseti	a0, zero, 0xb
80003d34: 30452073     	csrs	mie, a0
80003d38: 0141         	addi	sp, sp, 0x10
80003d3a: 8082         	ret

80003d3c <_start_rust>:
80003d3c: 1141         	addi	sp, sp, -0x10
80003d3e: c606         	sw	ra, 0xc(sp)
80003d40: 7ca02573     	csrr	a0, 0x7ca
80003d44: 00157593     	andi	a1, a0, 0x1
80003d48: c42a         	sw	a0, 0x8(sp)
80003d4a: e989         	bnez	a1, 0x80003d5c <_start_rust+0x20>
80003d4c: 7ca02573     	csrr	a0, 0x7ca
80003d50: c42a         	sw	a0, 0x8(sp)
80003d52: 30156513     	ori	a0, a0, 0x301
80003d56: c42a         	sw	a0, 0x8(sp)
80003d58: 7ca51073     	csrw	0x7ca, a0
80003d5c: 7ca02573     	csrr	a0, 0x7ca
80003d60: 00257593     	andi	a1, a0, 0x2
80003d64: c42a         	sw	a0, 0x8(sp)
80003d66: e19d         	bnez	a1, 0x80003d8c <_start_rust+0x50>
80003d68: 7ca02573     	csrr	a0, 0x7ca
80003d6c: c42a         	sw	a0, 0x8(sp)
80003d6e: 28d51513     	bseti	a0, a0, 0xd
80003d72: 28e51513     	bseti	a0, a0, 0xe
80003d76: c42a         	sw	a0, 0x8(sp)
80003d78: 7ca51073     	csrw	0x7ca, a0
80003d7c: 7ca02573     	csrr	a0, 0x7ca
80003d80: c42a         	sw	a0, 0x8(sp)
80003d82: 40256513     	ori	a0, a0, 0x402
80003d86: c42a         	sw	a0, 0x8(sp)
80003d88: 7ca51073     	csrw	0x7ca, a0
80003d8c: 455d         	li	a0, 0x17
80003d8e: c42a         	sw	a0, 0x8(sp)
80003d90: 7cc51073     	csrw	0x7cc, a0

80003d94 <.Lpcrel_hi3>:
80003d94: 7fffc517     	auipc	a0, 0x7fffc
80003d98: 26c50513     	addi	a0, a0, 0x26c

80003d9c <.Lpcrel_hi4>:
80003d9c: 7fffd597     	auipc	a1, 0x7fffd
80003da0: 95c58593     	addi	a1, a1, -0x6a4

80003da4 <.Lpcrel_hi5>:
80003da4: fffff617     	auipc	a2, 0xfffff
80003da8: 2ac60613     	addi	a2, a2, 0x2ac
80003dac: 00b57863     	bgeu	a0, a1, 0x80003dbc <.Lpcrel_hi6>
80003db0: 4214         	lw	a3, 0x0(a2)
80003db2: 0611         	addi	a2, a2, 0x4
80003db4: c114         	sw	a3, 0x0(a0)
80003db6: 0511         	addi	a0, a0, 0x4
80003db8: feb56ce3     	bltu	a0, a1, 0x80003db0 <.Lpcrel_hi5+0xc>

80003dbc <.Lpcrel_hi6>:
80003dbc: 8007c517     	auipc	a0, 0x8007c
80003dc0: 24450513     	addi	a0, a0, 0x244

80003dc4 <.Lpcrel_hi7>:
80003dc4: 8007c597     	auipc	a1, 0x8007c
80003dc8: 68c58593     	addi	a1, a1, 0x68c

80003dcc <.Lpcrel_hi8>:
80003dcc: 00000617     	auipc	a2, 0x0
80003dd0: 51c60613     	addi	a2, a2, 0x51c
80003dd4: 00b57863     	bgeu	a0, a1, 0x80003de4 <.Lpcrel_hi9>
80003dd8: 4214         	lw	a3, 0x0(a2)
80003dda: 0611         	addi	a2, a2, 0x4
80003ddc: c114         	sw	a3, 0x0(a0)
80003dde: 0511         	addi	a0, a0, 0x4
80003de0: feb56ce3     	bltu	a0, a1, 0x80003dd8 <.Lpcrel_hi8+0xc>

80003de4 <.Lpcrel_hi9>:
80003de4: 7fffd517     	auipc	a0, 0x7fffd
80003de8: 91450513     	addi	a0, a0, -0x6ec

80003dec <.Lpcrel_hi10>:
80003dec: 7fffd597     	auipc	a1, 0x7fffd
80003df0: 90c58593     	addi	a1, a1, -0x6f4

80003df4 <.Lpcrel_hi11>:
80003df4: 00000617     	auipc	a2, 0x0
80003df8: 54460613     	addi	a2, a2, 0x544
80003dfc: 00b57863     	bgeu	a0, a1, 0x80003e0c <.Lpcrel_hi12>
80003e00: 4214         	lw	a3, 0x0(a2)
80003e02: 0611         	addi	a2, a2, 0x4
80003e04: c114         	sw	a3, 0x0(a0)
80003e06: 0511         	addi	a0, a0, 0x4
80003e08: feb56ce3     	bltu	a0, a1, 0x80003e00 <.Lpcrel_hi11+0xc>

80003e0c <.Lpcrel_hi12>:
80003e0c: 8007c517     	auipc	a0, 0x8007c
80003e10: 64450513     	addi	a0, a0, 0x644

80003e14 <.Lpcrel_hi13>:
80003e14: 8007c597     	auipc	a1, 0x8007c
80003e18: 63c58593     	addi	a1, a1, 0x63c

80003e1c <.Lpcrel_hi14>:
80003e1c: 00000617     	auipc	a2, 0x0
80003e20: 51c60613     	addi	a2, a2, 0x51c
80003e24: 00b57863     	bgeu	a0, a1, 0x80003e34 <.Lpcrel_hi15>
80003e28: 4214         	lw	a3, 0x0(a2)
80003e2a: 0611         	addi	a2, a2, 0x4
80003e2c: c114         	sw	a3, 0x0(a0)
80003e2e: 0511         	addi	a0, a0, 0x4
80003e30: feb56ce3     	bltu	a0, a1, 0x80003e28 <.Lpcrel_hi14+0xc>

80003e34 <.Lpcrel_hi15>:
80003e34: 8007d517     	auipc	a0, 0x8007d
80003e38: 8a450513     	addi	a0, a0, -0x75c

80003e3c <.Lpcrel_hi16>:
80003e3c: 8007d597     	auipc	a1, 0x8007d
80003e40: 89c58593     	addi	a1, a1, -0x764

80003e44 <.Lpcrel_hi17>:
80003e44: 00000617     	auipc	a2, 0x0
80003e48: 4f460613     	addi	a2, a2, 0x4f4
80003e4c: 00b57863     	bgeu	a0, a1, 0x80003e5c <.Lpcrel_hi18>
80003e50: 4214         	lw	a3, 0x0(a2)
80003e52: 0611         	addi	a2, a2, 0x4
80003e54: c114         	sw	a3, 0x0(a0)
80003e56: 0511         	addi	a0, a0, 0x4
80003e58: feb56ce3     	bltu	a0, a1, 0x80003e50 <.Lpcrel_hi17+0xc>

80003e5c <.Lpcrel_hi18>:
80003e5c: 8007c517     	auipc	a0, 0x8007c
80003e60: 5f450513     	addi	a0, a0, 0x5f4

80003e64 <.Lpcrel_hi19>:
80003e64: 8007d597     	auipc	a1, 0x8007d
80003e68: 87458593     	addi	a1, a1, -0x78c
80003e6c: 00b57763     	bgeu	a0, a1, 0x80003e7a <.Lpcrel_hi20>
80003e70: 00052023     	sw	zero, 0x0(a0)
80003e74: 0511         	addi	a0, a0, 0x4
80003e76: feb56de3     	bltu	a0, a1, 0x80003e70 <.Lpcrel_hi19+0xc>

80003e7a <.Lpcrel_hi20>:
80003e7a: 8007c517     	auipc	a0, 0x8007c
80003e7e: 5d650513     	addi	a0, a0, 0x5d6

80003e82 <.Lpcrel_hi21>:
80003e82: 8007c597     	auipc	a1, 0x8007c
80003e86: 5ce58593     	addi	a1, a1, 0x5ce
80003e8a: 00b57763     	bgeu	a0, a1, 0x80003e98 <.Lpcrel_hi22>
80003e8e: 00052023     	sw	zero, 0x0(a0)
80003e92: 0511         	addi	a0, a0, 0x4
80003e94: feb56de3     	bltu	a0, a1, 0x80003e8e <.Lpcrel_hi21+0xc>

80003e98 <.Lpcrel_hi22>:
80003e98: 8007d517     	auipc	a0, 0x8007d
80003e9c: 84050513     	addi	a0, a0, -0x7c0

80003ea0 <.Lpcrel_hi23>:
80003ea0: 8007d597     	auipc	a1, 0x8007d
80003ea4: 83858593     	addi	a1, a1, -0x7c8
80003ea8: 00b57763     	bgeu	a0, a1, 0x80003eb6 <.Lpcrel_hi23+0x16>
80003eac: 00052023     	sw	zero, 0x0(a0)
80003eb0: 0511         	addi	a0, a0, 0x4
80003eb2: feb56de3     	bltu	a0, a1, 0x80003eac <.Lpcrel_hi23+0xc>
80003eb6: 00000097     	auipc	ra, 0x0
80003eba: e0e080e7     	jalr	-0x1f2(ra) <_setup_interrupts>
80003ebe: 7d002573     	csrr	a0, 0x7d0
80003ec2: c42a         	sw	a0, 0x8(sp)
80003ec4: 00156513     	ori	a0, a0, 0x1
80003ec8: c42a         	sw	a0, 0x8(sp)
80003eca: 7d051073     	csrw	0x7d0, a0
80003ece: 30002573     	csrr	a0, mstatus
80003ed2: 75e9         	lui	a1, 0xffffa
80003ed4: 15fd         	addi	a1, a1, -0x1
80003ed6: 8d6d         	and	a0, a0, a1
80003ed8: 28e51513     	bseti	a0, a0, 0xe
80003edc: 30051073     	csrw	mstatus, a0
80003ee0: 30002573     	csrr	a0, mstatus
80003ee4: 8d6d         	and	a0, a0, a1
80003ee6: 28d51513     	bseti	a0, a0, 0xd
80003eea: 30051073     	csrw	mstatus, a0
80003eee: 00000097     	auipc	ra, 0x0
80003ef2: 8ae080e7     	jalr	-0x752(ra) <main>

80003ef6 <OUTLINED_FUNCTION_0>:
80003ef6: 1101         	addi	sp, sp, -0x20
80003ef8: ce06         	sw	ra, 0x1c(sp)
80003efa: cc22         	sw	s0, 0x18(sp)
80003efc: ca26         	sw	s1, 0x14(sp)
80003efe: c84a         	sw	s2, 0x10(sp)
80003f00: c64e         	sw	s3, 0xc(sp)
80003f02: 8282         	jr	t0

80003f04 <__udivdi3>:
80003f04: 00000317     	auipc	t1, 0x0
80003f08: 1e230067     	jr	0x1e2(t1) <compiler_builtins::int::udiv::__udivdi3::h9e9a45df01a74111>

80003f0c <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c>:
80003f0c: 1101         	addi	sp, sp, -0x20
80003f0e: ce06         	sw	ra, 0x1c(sp)
80003f10: cc22         	sw	s0, 0x18(sp)
80003f12: ca26         	sw	s1, 0x14(sp)
80003f14: c84a         	sw	s2, 0x10(sp)
80003f16: c64e         	sw	s3, 0xc(sp)
80003f18: c452         	sw	s4, 0x8(sp)
80003f1a: c256         	sw	s5, 0x4(sp)
80003f1c: c05a         	sw	s6, 0x0(sp)
80003f1e: 8a3a         	mv	s4, a4
80003f20: 89b6         	mv	s3, a3
80003f22: 8b32         	mv	s6, a2
80003f24: 84ae         	mv	s1, a1
80003f26: 892a         	mv	s2, a0
80003f28: c725         	beqz	a4, 0x80003f90 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x84>
80003f2a: 4681         	li	a3, 0x0
80003f2c: 080b0363     	beqz	s6, 0x80003fb2 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0xa6>
80003f30: 074b0e63     	beq	s6, s4, 0x80003fac <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0xa0>
80003f34: 014b3533     	sltu	a0, s6, s4
80003f38: ed2d         	bnez	a0, 0x80003fb2 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0xa6>
80003f3a: 600a1513     	clz	a0, s4
80003f3e: 600b1593     	clz	a1, s6
80003f42: 40b50ab3     	sub	s5, a0, a1
80003f46: 03faf613     	andi	a2, s5, 0x3f
80003f4a: 854e         	mv	a0, s3
80003f4c: 85d2         	mv	a1, s4
80003f4e: 00000097     	auipc	ra, 0x0
80003f52: 190080e7     	jalr	0x190(ra) <__ashldi3>
80003f56: 4681         	li	a3, 0x0
80003f58: 29501733     	bset	a4, zero, s5
80003f5c: 00a4b633     	sltu	a2, s1, a0
80003f60: 40bb07b3     	sub	a5, s6, a1
80003f64: 40c78433     	sub	s0, a5, a2
80003f68: 00044d63     	bltz	s0, 0x80003f82 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x76>
80003f6c: 8c89         	sub	s1, s1, a0
80003f6e: 01440563     	beq	s0, s4, 0x80003f78 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x6c>
80003f72: 014437b3     	sltu	a5, s0, s4
80003f76: a019         	j	0x80003f7c <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x70>
80003f78: 0134b7b3     	sltu	a5, s1, s3
80003f7c: 8ed9         	or	a3, a3, a4
80003f7e: e78d         	bnez	a5, 0x80003fa8 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x9c>
80003f80: 8b22         	mv	s6, s0
80003f82: 8105         	srli	a0, a0, 0x1
80003f84: 01f59613     	slli	a2, a1, 0x1f
80003f88: 8d51         	or	a0, a0, a2
80003f8a: 8185         	srli	a1, a1, 0x1
80003f8c: 8305         	srli	a4, a4, 0x1
80003f8e: b7f9         	j	0x80003f5c <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x50>
80003f90: 020b0463     	beqz	s6, 0x80003fb8 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0xac>
80003f94: 033b7963     	bgeu	s6, s3, 0x80003fc6 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0xba>
80003f98: 600b1513     	clz	a0, s6
80003f9c: 60099593     	clz	a1, s3
80003fa0: 02a59c63     	bne	a1, a0, 0x80003fd8 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0xcc>
80003fa4: 4afd         	li	s5, 0x1f
80003fa6: a825         	j	0x80003fde <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0xd2>
80003fa8: 4801         	li	a6, 0x0
80003faa: a8b5         	j	0x80004026 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x11a>
80003fac: 0134b533     	sltu	a0, s1, s3
80003fb0: d549         	beqz	a0, 0x80003f3a <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x2e>
80003fb2: 8836         	mv	a6, a3
80003fb4: 845a         	mv	s0, s6
80003fb6: a885         	j	0x80004026 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x11a>
80003fb8: 4801         	li	a6, 0x0
80003fba: 4401         	li	s0, 0x0
80003fbc: 0334d6b3     	divu	a3, s1, s3
80003fc0: 0334f4b3     	remu	s1, s1, s3
80003fc4: a08d         	j	0x80004026 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x11a>
80003fc6: 093b1063     	bne	s6, s3, 0x80004046 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x13a>
80003fca: 4401         	li	s0, 0x0
80003fcc: 0364d6b3     	divu	a3, s1, s6
80003fd0: 0364f4b3     	remu	s1, s1, s6
80003fd4: 4805         	li	a6, 0x1
80003fd6: a881         	j	0x80004026 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x11a>
80003fd8: 8d89         	sub	a1, a1, a0
80003fda: 02058a93     	addi	s5, a1, 0x20
80003fde: 854e         	mv	a0, s3
80003fe0: 85d2         	mv	a1, s4
80003fe2: 8656         	mv	a2, s5
80003fe4: 00000097     	auipc	ra, 0x0
80003fe8: 0fa080e7     	jalr	0xfa(ra) <__ashldi3>
80003fec: 4681         	li	a3, 0x0
80003fee: 29501633     	bset	a2, zero, s5
80003ff2: 00a4b733     	sltu	a4, s1, a0
80003ff6: 40bb07b3     	sub	a5, s6, a1
80003ffa: 40e78733     	sub	a4, a5, a4
80003ffe: 00074663     	bltz	a4, 0x8000400a <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0xfe>
80004002: 8c89         	sub	s1, s1, a0
80004004: 8ed1         	or	a3, a3, a2
80004006: cb09         	beqz	a4, 0x80004018 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x10c>
80004008: 8b3a         	mv	s6, a4
8000400a: 8105         	srli	a0, a0, 0x1
8000400c: 01f59713     	slli	a4, a1, 0x1f
80004010: 8d59         	or	a0, a0, a4
80004012: 8185         	srli	a1, a1, 0x1
80004014: 8205         	srli	a2, a2, 0x1
80004016: bff1         	j	0x80003ff2 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0xe6>
80004018: 4801         	li	a6, 0x0
8000401a: 4401         	li	s0, 0x0
8000401c: 0334d533     	divu	a0, s1, s3
80004020: 0334f4b3     	remu	s1, s1, s3
80004024: 8ec9         	or	a3, a3, a0
80004026: 854a         	mv	a0, s2
80004028: c114         	sw	a3, 0x0(a0)
8000402a: 01092223     	sw	a6, 0x4(s2)
8000402e: c504         	sw	s1, 0x8(a0)
80004030: c540         	sw	s0, 0xc(a0)
80004032: 40f2         	lw	ra, 0x1c(sp)
80004034: 4462         	lw	s0, 0x18(sp)
80004036: 44d2         	lw	s1, 0x14(sp)
80004038: 4942         	lw	s2, 0x10(sp)
8000403a: 49b2         	lw	s3, 0xc(sp)
8000403c: 4a22         	lw	s4, 0x8(sp)
8000403e: 4a92         	lw	s5, 0x4(sp)
80004040: 4b02         	lw	s6, 0x0(sp)
80004042: 6105         	addi	sp, sp, 0x20
80004044: 8082         	ret
80004046: 0109d593     	srli	a1, s3, 0x10
8000404a: 033b5833     	divu	a6, s6, s3
8000404e: 033b7433     	remu	s0, s6, s3
80004052: e995         	bnez	a1, 0x80004086 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x17a>
80004054: 0104d593     	srli	a1, s1, 0x10
80004058: 01041613     	slli	a2, s0, 0x10
8000405c: 8dd1         	or	a1, a1, a2
8000405e: 0335d633     	divu	a2, a1, s3
80004062: 0335f5b3     	remu	a1, a1, s3
80004066: 0804c6b3     	zext.h	a3, s1
8000406a: 05c2         	slli	a1, a1, 0x10
8000406c: 8dd5         	or	a1, a1, a3
8000406e: 0335d6b3     	divu	a3, a1, s3
80004072: 0335f4b3     	remu	s1, a1, s3
80004076: 01061593     	slli	a1, a2, 0x10
8000407a: 8241         	srli	a2, a2, 0x10
8000407c: 01066833     	or	a6, a2, a6
80004080: 8ecd         	or	a3, a3, a1
80004082: 4401         	li	s0, 0x0
80004084: b74d         	j	0x80004026 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x11a>
80004086: 01440563     	beq	s0, s4, 0x80004090 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x184>
8000408a: 014435b3     	sltu	a1, s0, s4
8000408e: a019         	j	0x80004094 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x188>
80004090: 0134b5b3     	sltu	a1, s1, s3
80004094: c199         	beqz	a1, 0x8000409a <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x18e>
80004096: 4681         	li	a3, 0x0
80004098: b779         	j	0x80004026 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x11a>
8000409a: 0019d693     	srli	a3, s3, 0x1
8000409e: 0a7e         	slli	s4, s4, 0x1f
800040a0: 00da66b3     	or	a3, s4, a3
800040a4: 01f99793     	slli	a5, s3, 0x1f
800040a8: 80000737     	lui	a4, 0x80000
800040ac: 00f4b533     	sltu	a0, s1, a5
800040b0: 40d40633     	sub	a2, s0, a3
800040b4: 8e09         	sub	a2, a2, a0
800040b6: 00064663     	bltz	a2, 0x800040c2 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x1b6>
800040ba: 8c9d         	sub	s1, s1, a5
800040bc: 8dd9         	or	a1, a1, a4
800040be: ca09         	beqz	a2, 0x800040d0 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x1c4>
800040c0: 8432         	mv	s0, a2
800040c2: 8385         	srli	a5, a5, 0x1
800040c4: 01f69513     	slli	a0, a3, 0x1f
800040c8: 8fc9         	or	a5, a5, a0
800040ca: 8285         	srli	a3, a3, 0x1
800040cc: 8305         	srli	a4, a4, 0x1
800040ce: bff9         	j	0x800040ac <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x1a0>
800040d0: 4401         	li	s0, 0x0
800040d2: 0334d6b3     	divu	a3, s1, s3
800040d6: 0334f4b3     	remu	s1, s1, s3
800040da: 8ecd         	or	a3, a3, a1
800040dc: b7a9         	j	0x80004026 <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c+0x11a>

800040de <__ashldi3>:
800040de: 00000317     	auipc	t1, 0x0
800040e2: 0ae30067     	jr	0xae(t1) <compiler_builtins::int::shift::__ashldi3::h74bfb44cf04c92e2>

800040e6 <compiler_builtins::int::udiv::__udivdi3::h9e9a45df01a74111>:
800040e6: 1101         	addi	sp, sp, -0x20
800040e8: ce06         	sw	ra, 0x1c(sp)
800040ea: 8736         	mv	a4, a3
800040ec: 86b2         	mv	a3, a2
800040ee: 862e         	mv	a2, a1
800040f0: 85aa         	mv	a1, a0
800040f2: 0028         	addi	a0, sp, 0x8
800040f4: 00000097     	auipc	ra, 0x0
800040f8: e18080e7     	jalr	-0x1e8(ra) <compiler_builtins::int::specialized_div_rem::u64_div_rem::h8061304a41e6774c>
800040fc: 45b2         	lw	a1, 0xc(sp)
800040fe: 4522         	lw	a0, 0x8(sp)
80004100: 40f2         	lw	ra, 0x1c(sp)
80004102: 6105         	addi	sp, sp, 0x20
80004104: 8082         	ret

80004106 <<u32 as compiler_builtins::int::Int>::logical_shr::h1d60a567480f7e70>:
80004106: 00b55533     	srl	a0, a0, a1
8000410a: 8082         	ret

8000410c <<u32 as compiler_builtins::int::Int>::wrapping_shl::hc51104d134b1d1c8>:
8000410c: 00b51533     	sll	a0, a0, a1
80004110: 8082         	ret

80004112 <compiler_builtins::int::DInt::from_lo_hi::h2bbdcc30deee6662>:
80004112: 8082         	ret

80004114 <compiler_builtins::int::shift::Ashl::ashl::h3f4d0723e1580fd2>:
80004114: 00000297     	auipc	t0, 0x0
80004118: 140282e7     	jalr	t0, 0x140(t0) <OUTLINED_FUNCTION_6>
8000411c: 84b2         	mv	s1, a2
8000411e: 02067613     	andi	a2, a2, 0x20
80004122: 842a         	mv	s0, a0
80004124: e621         	bnez	a2, 0x8000416c <compiler_builtins::int::shift::Ashl::ashl::h3f4d0723e1580fd2+0x58>
80004126: 892e         	mv	s2, a1
80004128: c8b9         	beqz	s1, 0x8000417e <compiler_builtins::int::shift::Ashl::ashl::h3f4d0723e1580fd2+0x6a>
8000412a: 8522         	mv	a0, s0
8000412c: 85a6         	mv	a1, s1
8000412e: 00000097     	auipc	ra, 0x0
80004132: fde080e7     	jalr	-0x22(ra) <<u32 as compiler_builtins::int::Int>::wrapping_shl::hc51104d134b1d1c8>
80004136: 89aa         	mv	s3, a0
80004138: 02000593     	li	a1, 0x20
8000413c: 8d85         	sub	a1, a1, s1
8000413e: 8522         	mv	a0, s0
80004140: 00000097     	auipc	ra, 0x0
80004144: fc6080e7     	jalr	-0x3a(ra) <<u32 as compiler_builtins::int::Int>::logical_shr::h1d60a567480f7e70>
80004148: 842a         	mv	s0, a0
8000414a: 854a         	mv	a0, s2
8000414c: 85a6         	mv	a1, s1
8000414e: 00000097     	auipc	ra, 0x0
80004152: fbe080e7     	jalr	-0x42(ra) <<u32 as compiler_builtins::int::Int>::wrapping_shl::hc51104d134b1d1c8>
80004156: 008565b3     	or	a1, a0, s0
8000415a: 854e         	mv	a0, s3
8000415c: 00000297     	auipc	t0, 0x0
80004160: 0ea282e7     	jalr	t0, 0xea(t0) <OUTLINED_FUNCTION_5>
80004164: 00000317     	auipc	t1, 0x0
80004168: fae30067     	jr	-0x52(t1) <compiler_builtins::int::DInt::from_lo_hi::h2bbdcc30deee6662>
8000416c: fe048593     	addi	a1, s1, -0x20
80004170: 8522         	mv	a0, s0
80004172: 00000097     	auipc	ra, 0x0
80004176: f9a080e7     	jalr	-0x66(ra) <<u32 as compiler_builtins::int::Int>::wrapping_shl::hc51104d134b1d1c8>
8000417a: 892a         	mv	s2, a0
8000417c: 4401         	li	s0, 0x0
8000417e: 8522         	mv	a0, s0
80004180: 85ca         	mv	a1, s2
80004182: 00000297     	auipc	t0, 0x0
80004186: 0c4282e7     	jalr	t0, 0xc4(t0) <OUTLINED_FUNCTION_5>
8000418a: 8082         	ret

8000418c <compiler_builtins::int::shift::__ashldi3::h74bfb44cf04c92e2>:
8000418c: 00000317     	auipc	t1, 0x0
80004190: f8830067     	jr	-0x78(t1) <compiler_builtins::int::shift::Ashl::ashl::h3f4d0723e1580fd2>

80004194 <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb>:
80004194: 46c1         	li	a3, 0x10
80004196: 08d66063     	bltu	a2, a3, 0x80004216 <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb+0x82>
8000419a: 40a006b3     	neg	a3, a0
8000419e: 0036f813     	andi	a6, a3, 0x3
800041a2: 010503b3     	add	t2, a0, a6
800041a6: 87ae         	mv	a5, a1
800041a8: 86aa         	mv	a3, a0
800041aa: 00757a63     	bgeu	a0, t2, 0x800041be <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb+0x2a>
800041ae: 0007c703     	lbu	a4, 0x0(a5)
800041b2: 00e68023     	sb	a4, 0x0(a3)
800041b6: 0685         	addi	a3, a3, 0x1
800041b8: 0785         	addi	a5, a5, 0x1
800041ba: fe76eae3     	bltu	a3, t2, 0x800041ae <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb+0x1a>
800041be: 010582b3     	add	t0, a1, a6
800041c2: 41060833     	sub	a6, a2, a6
800041c6: ffc87893     	andi	a7, a6, -0x4
800041ca: 0032f593     	andi	a1, t0, 0x3
800041ce: 011386b3     	add	a3, t2, a7
800041d2: cdb9         	beqz	a1, 0x80004230 <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb+0x9c>
800041d4: 00329593     	slli	a1, t0, 0x3
800041d8: 0185f313     	andi	t1, a1, 0x18
800041dc: ffc2f713     	andi	a4, t0, -0x4
800041e0: 4310         	lw	a2, 0x0(a4)
800041e2: 40b007b3     	neg	a5, a1
800041e6: 0187fe13     	andi	t3, a5, 0x18
800041ea: 00470593     	addi	a1, a4, 0x4
800041ee: 00d3ff63     	bgeu	t2, a3, 0x8000420c <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb+0x78>
800041f2: 4198         	lw	a4, 0x0(a1)
800041f4: 00665633     	srl	a2, a2, t1
800041f8: 01c717b3     	sll	a5, a4, t3
800041fc: 8e5d         	or	a2, a2, a5
800041fe: 00c3a023     	sw	a2, 0x0(t2)
80004202: 0391         	addi	t2, t2, 0x4
80004204: 0591         	addi	a1, a1, 0x4
80004206: 863a         	mv	a2, a4
80004208: fed3e5e3     	bltu	t2, a3, 0x800041f2 <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb+0x5e>
8000420c: 011285b3     	add	a1, t0, a7
80004210: 00387613     	andi	a2, a6, 0x3
80004214: a011         	j	0x80004218 <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb+0x84>
80004216: 86aa         	mv	a3, a0
80004218: 9636         	add	a2, a2, a3
8000421a: 00c6fa63     	bgeu	a3, a2, 0x8000422e <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb+0x9a>
8000421e: 0005c703     	lbu	a4, 0x0(a1)
80004222: 00e68023     	sb	a4, 0x0(a3)
80004226: 0685         	addi	a3, a3, 0x1
80004228: 0585         	addi	a1, a1, 0x1
8000422a: fec6eae3     	bltu	a3, a2, 0x8000421e <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb+0x8a>
8000422e: 8082         	ret
80004230: 8596         	mv	a1, t0
80004232: fcd3fde3     	bgeu	t2, a3, 0x8000420c <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb+0x78>
80004236: 4190         	lw	a2, 0x0(a1)
80004238: 00c3a023     	sw	a2, 0x0(t2)
8000423c: 0391         	addi	t2, t2, 0x4
8000423e: 0591         	addi	a1, a1, 0x4
80004240: fed3ebe3     	bltu	t2, a3, 0x80004236 <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb+0xa2>
80004244: b7e1         	j	0x8000420c <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb+0x78>

80004246 <OUTLINED_FUNCTION_5>:
80004246: 40f2         	lw	ra, 0x1c(sp)
80004248: 4462         	lw	s0, 0x18(sp)
8000424a: 44d2         	lw	s1, 0x14(sp)
8000424c: 4942         	lw	s2, 0x10(sp)
8000424e: 49b2         	lw	s3, 0xc(sp)
80004250: 6105         	addi	sp, sp, 0x20
80004252: 8282         	jr	t0

80004254 <OUTLINED_FUNCTION_6>:
80004254: 1101         	addi	sp, sp, -0x20
80004256: ce06         	sw	ra, 0x1c(sp)
80004258: cc22         	sw	s0, 0x18(sp)
8000425a: ca26         	sw	s1, 0x14(sp)
8000425c: c84a         	sw	s2, 0x10(sp)
8000425e: c64e         	sw	s3, 0xc(sp)
80004260: 8282         	jr	t0

80004262 <memcpy>:
80004262: 00000317     	auipc	t1, 0x0
80004266: f3230067     	jr	-0xce(t1) <compiler_builtins::mem::memcpy::h97e5ee5777ff3cdb>
8000426a: 0000         	unimp
8000426c: 0000         	unimp
8000426e: 0000         	unimp
