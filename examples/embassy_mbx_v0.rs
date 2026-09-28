#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(abi_riscv_interrupt)]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write as _;
use futures_util::StreamExt;
use hal::gpio::{Input, Level, Output, Pull};
use hal::mbx::Mbx;
use hal::{bind_interrupts, peripherals};
use hpm_hal as hal;

bind_interrupts!(struct Irqs {
    MBX0A => hal::mbx::InterruptHandler<peripherals::MBX0A>;
    MBX0B => hal::mbx::InterruptHandler<peripherals::MBX0B>;
});

#[embassy_executor::task]
async fn mailbox(mbox: Mbx<'static, peripherals::MBX0A>) {
    let mut mbox = mbox;
    let mut i = 345;
    loop {
        defmt::info!("[task0] sending {}", i);
        mbox.send_fifo(i).await;
        i += 1;

        Timer::after_millis(100).await;
    }
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    let mut led = Output::new(p.PA10, Level::Low, Default::default());

    let inbox = Mbx::new(p.MBX0A, Irqs);
    let mut outbox = Mbx::new(p.MBX0B, Irqs);

    spawner.spawn(mailbox(inbox)).unwrap();
    defmt::info!("Mailbox task spawned!");

    /*
    inbox.blocking_send(114514);

    let data = outbox.blocking_receive();

    writeln!(uart, "Received: {}", data).unwrap();
    */

    // inbox.send(233_3333).await;

    //let data = outbox.receive().await;

    //defmt::info!("[main] Received: {}", data);

    // writeln!(uart, "Received: {}", data).unwrap();

    loop {
        led.toggle();

        // let data = outbox.receive().await;

        while let Some(val) = outbox.next().await {
            defmt::info!("[main] Received: {}", val);
            Timer::after_millis(1000).await; // slower
        }

        // defmt::info!("[main] Received: {}", data);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    //let _ = println!("\n\n\n{}", info);

    defmt::info!("Panic!");

    loop {}
}
