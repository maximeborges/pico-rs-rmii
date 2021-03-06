#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::*;
use panic_probe as _;
use rp2040_hal as hal;

mod pio;
use crate::pio::{init_eth, EthPins};

use hal::{
    clocks::*,
    pac,
    pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking, PLLConfig},
    sio::Sio,
    watchdog::Watchdog,
    xosc::setup_xosc_blocking,
};

#[entry]
fn main() -> ! {
    println!("Program start");

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);

    let clocks = setup_clocks(
        pac.XOSC,
        pac.PLL_SYS,
        pac.PLL_USB,
        pac.CLOCKS,
        &mut pac.RESETS,
        pac.WATCHDOG,
    );
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let eth_pins = EthPins {
        ref_clk: pins.gpio21.into(),
        md_io: pins.gpio14.into(),
        md_clk: pins.gpio15.into(),
        // Those 3 pins should be one after the other
        rx_d0: pins.gpio6.into(),
        rx_d1: pins.gpio7.into(),
        crs: pins.gpio8.into(),
        // Those 3 pins should be one after the other
        tx_d0: pins.gpio10.into(),
        tx_d1: pins.gpio11.into(),
        tx_en: pins.gpio12.into(),
    };

    init_eth(eth_pins, pac.PIO0, pac.DMA, &mut pac.RESETS);

    let mut led_pin = pins.gpio25.into_push_pull_output();
    loop {
        info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

fn setup_clocks(
    xosc: pac::XOSC,
    pll_sys: pac::PLL_SYS,
    pll_usb: pac::PLL_USB,
    clocks: pac::CLOCKS,
    resets: &mut pac::RESETS,
    watchdog: pac::WATCHDOG,
) -> ClocksManager {
    let xosc_crystal_freq = 12_000_000u32;
    let xosc = setup_xosc_blocking(xosc, xosc_crystal_freq.Hz()).unwrap_or_else(|_| {
        error!("Xosc failed to start");
        loop {}
    });

    // Configure watchdog tick generation to tick over every microsecond
    let mut watchdog = Watchdog::new(watchdog);
    watchdog.enable_tick_generation((xosc_crystal_freq / 1_000_000) as u8);

    // External clock from RMII module
    let clock_gpio_freq = 50_000_000u32;

    let mut clocks: ClocksManager = ClocksManager::new(clocks);

    pub const PLL_SYS_100MHZ: PLLConfig<Megahertz> = PLLConfig {
        vco_freq: Megahertz(1500),
        refdiv: 1,
        post_div1: 5,
        post_div2: 3,
    };

    let pll_sys = setup_pll_blocking(
        pll_sys,
        xosc.operating_frequency().into(),
        PLL_SYS_100MHZ,
        &mut clocks,
        resets,
    )
    .unwrap_or_else(|_| {
        error!("Failed to start SYS PLL");
        // led_pin.set_high().unwrap();
        loop {}
    });

    let pll_usb = setup_pll_blocking(
        pll_usb,
        xosc.operating_frequency().into(),
        PLL_USB_48MHZ,
        &mut clocks,
        resets,
    )
    .unwrap_or_else(|_| {
        error!("Failed to start USB PLL");
        loop {}
    });

    let clocks = (|| {
        // CLK_REF = XOSC (12MHz) / 1 = 12MHz
        clocks
            .reference_clock
            .configure_clock(&xosc, xosc.get_freq())?;

        // CLK SYS = PLL SYS (100MHz) / 1 = 100MHz
        clocks
            .system_clock
            .configure_clock(&pll_sys, pll_sys.get_freq())?;

        // CLK USB = PLL USB (48MHz) / 1 = 48MHz
        clocks
            .usb_clock
            .configure_clock(&pll_usb, pll_usb.get_freq())?;

        // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
        clocks
            .adc_clock
            .configure_clock(&pll_usb, pll_usb.get_freq())?;

        // CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
        clocks.rtc_clock.configure_clock(&pll_usb, 46875u32.Hz())?;

        // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
        // Normally choose clk_sys or clk_usb
        clocks
            .peripheral_clock
            .configure_clock(&clocks.system_clock, clock_gpio_freq.Hz())?;

        clocks
            .gpio_output0_clock
            .configure_clock(&clocks.system_clock, clock_gpio_freq.Hz())?;

        Ok(clocks)
    })()
    .unwrap_or_else(|_: ClockError| {
        error!("Failed to set clocks");
        loop {}
    });

    clocks
}

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;
