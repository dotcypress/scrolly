#![no_std]
#![no_main]

extern crate panic_probe;
extern crate rp2040_hal as hal;
extern crate rtic;

use defmt_rtt as _;

use cortex_m::singleton;
use embedded_hal::digital::v2::*;
use hal::gpio::*;
use hal::pac;
use hal::timer::{monotonic::Monotonic, *};
use hal::usb::UsbBus;
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::MouseReport;
use usbd_hid::hid_class::HIDClass;

pub const XTAL_FREQ_HZ: u32 = 12_000_000_u32;

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[rtic::app(device = pac, peripherals = true)]
mod app {
    use super::*;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Oracle = Monotonic<Alarm0>;

    #[local]
    struct Local {
        usb_dev: UsbDevice<'static, hal::usb::UsbBus>,
        dir_pin: Pin<bank0::Gpio10, FunctionSio<SioInput>, PullUp>,
        pulse_pin: Pin<bank0::Gpio8, FunctionSio<SioInput>, PullUp>,
    }

    #[shared]
    struct Shared {
        usb_hid: HIDClass<'static, hal::usb::UsbBus>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = ctx.device.RESETS;
        let mut watchdog = hal::Watchdog::new(ctx.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .expect("Clocks init failed");

        let sio = hal::Sio::new(ctx.device.SIO);
        let pins = Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        pins.gpio9.into_push_pull_output_in_state(PinState::Low);
        let dir_pin = pins.gpio10.into_pull_up_input();

        let pulse_pin = pins.gpio8.into_pull_up_input();
        pulse_pin.set_interrupt_enabled(Interrupt::EdgeLow, true);

        let mut timer = hal::Timer::new(ctx.device.TIMER, &mut resets, &clocks);
        let alarm = timer.alarm_0().expect("Alarm0 init failed");
        let mono = Monotonic::new(timer, alarm);

        let usb_regs = ctx.device.USBCTRL_REGS;
        let usb_dpram = ctx.device.USBCTRL_DPRAM;
        let usb_bus = UsbBus::new(usb_regs, usb_dpram, clocks.usb_clock, true, &mut resets);
        let usb_bus: &'static UsbBusAllocator<UsbBus> =
            singleton!(: UsbBusAllocator<UsbBus> = UsbBusAllocator::new(usb_bus))
                .expect("USB init failed");

        let usb_hid = HIDClass::new(usb_bus, MouseReport::desc(), 60);
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0xdcba))
            .manufacturer("vitaly.codes")
            .product("Scrolly")
            .serial_number("42")
            .build();

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
            pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
        };

        (
            Shared { usb_hid },
            Local {
                usb_dev,
                dir_pin,
                pulse_pin,
            },
            init::Monotonics(mono),
        )
    }

    #[task(binds = USBCTRL_IRQ, priority = 2, local = [usb_dev], shared = [usb_hid])]
    fn usb_irq(ctx: usb_irq::Context) {
        let mut usb_hid = ctx.shared.usb_hid;
        usb_hid.lock(|hid| {
            ctx.local.usb_dev.poll(&mut [hid]);
        });
    }

    #[task(binds = IO_IRQ_BANK0, priority = 1, local = [dir_pin, pulse_pin], shared = [usb_hid])]
    fn io_irq(ctx: io_irq::Context) {
        let io_irq::LocalResources { dir_pin, pulse_pin } = ctx.local;
        pulse_pin.clear_interrupt(Interrupt::EdgeLow);

        let wheel = if dir_pin.is_high().unwrap_or_default() {
            1
        } else {
            -1
        };

        let report = MouseReport {
            buttons: 0,
            x: 0,
            y: 0,
            pan: 0,
            wheel,
        };

        let mut usb_hid = ctx.shared.usb_hid;
        usb_hid.lock(|hid| {
            hid.push_input(&report).ok();
        });
    }
}
