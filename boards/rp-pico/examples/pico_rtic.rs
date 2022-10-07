#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [XIP_IRQ, USBCTRL_IRQ, PWM_IRQ_WRAP, ADC_IRQ_FIFO])]
mod app {

    use embedded_hal::digital::v2::OutputPin;
    use fugit::{ExtU64, MicrosDurationU32};
    use rp_pico::{
        hal::{self, clocks::init_clocks_and_plls, timer::Alarm, watchdog::Watchdog, Sio},
        XOSC_CRYSTAL_FREQ,
    };

    defmt::timestamp!("{:?}", monotonics::now());

    // systick monotonic
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = systick_monotonic::Systick<1_000>;

    // rp2040_monotonic
    // #[monotonic(binds = TIMER_IRQ_0, default = true)]
    // type MyMono = rp2040_monotonic::Rp2040Monotonic;

    // rp-hal monotonic
    // #[monotonic(binds = TIMER_IRQ_0, default = true)]
    // type MyMono = hal::timer::monotonic::Monotonic<hal::timer::Alarm0>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let _clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        blink::spawn().ok();
        foo::spawn().ok();

        // systick monotonic
        let monotonic = systick_monotonic::Systick::new(c.core.SYST, 125_000_000);

        // rp2040_monotonic
        // let monotonic = rp2040_monotonic::Rp2040Monotonic::new(c.device.TIMER);

        // rp-hal monotonic
        // let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
        // let alarm = timer.alarm_0().unwrap();
        // let monotonic = hal::timer::monotonic::Monotonic::new(timer, alarm);

        (Shared {}, Local {}, init::Monotonics(monotonic))
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi(); // put the MCU in sleep mode until interrupt occurs
        }
    }

    #[task]
    async fn foo(_cx: foo::Context) {
        let task1 = async {
            let mut i = 0;
            loop {
                if i == 500 {
                    break;
                }
                defmt::info!("hello from task 1 {}", i);
                monotonics::delay(10.millis()).await;
                i += 1;
            }
        };

        let task2 = async {
            let mut i = 0;
            loop {
                if i == 50 {
                    break;
                }
                defmt::info!("hello from task 2 {}", i);
                monotonics::delay(100.millis()).await;
                i += 1;
            }
        };

        futures::join!(task1, task2);
    }

    #[task]
    fn blink(ctx: blink::Context) {
        defmt::info!("hello from blink");
        blink::spawn_after(500.millis()).ok();
    }
}
