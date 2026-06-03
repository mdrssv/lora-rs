//! This example runs on the STM32WL board, which has a builtin Semtech Sx1262 radio.
//! It demonstrates LORA P2P receive functionality in conjunction with the lora_p2p_send example.
#![no_std]
#![no_main]

#[path = "../iv.rs"]
mod iv;

use defmt::{warn, info};
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_time::{Delay, Timer};
use embassy_stm32::mode::Async;
use lora_phy::sx126x::{Stm32wl, Sx126x, TcxoCtrlVoltage};
use lora_phy::mod_traits::InterfaceVariant;
use lora_phy::{mod_params::*, sx126x};
use lora_phy::{LoRa, RxMode};
use {defmt_rtt as _, panic_probe as _};

use self::iv::{InterruptHandler, Stm32wlInterfaceVariant, SubghzSpiDevice};

const LORA_FREQUENCY_IN_HZ: u32 = 868_000_000; // warning: set this appropriately for the region

bind_interrupts!(struct Irqs{
    SUBGHZ_RADIO => InterruptHandler;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(32_000_000),
            mode: HseMode::Bypass,
            prescaler: HsePrescaler::DIV1,
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL6,
            divp: None,
            divq: Some(PllQDiv::DIV2), // PLL1_Q clock (32 / 2 * 6 / 2), used for RNG
            divr: Some(PllRDiv::DIV2), // sysclk 48Mhz clock (32 / 2 * 6 / 2)
        });
    }
    let p = embassy_stm32::init(config);

    let ctrl1 = Output::new(p.PC4.degrade(), Level::Low, Speed::High);
    let ctrl2 = Output::new(p.PC5.degrade(), Level::Low, Speed::High);
    let ctrl3 = Output::new(p.PC3.degrade(), Level::High, Speed::High);

    let spi = Spi::new_subghz(p.SUBGHZSPI, p.DMA1_CH1, p.DMA1_CH2);
    let spi = SubghzSpiDevice(spi);
    let use_high_power_pa = true;
    let config = sx126x::Config {
        chip: Stm32wl { use_high_power_pa },
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
        use_dcdc: true,
        rx_boost: false,
    };
    let mut iv = Stm32wlInterfaceVariant::new(Irqs, use_high_power_pa, Some(ctrl1), Some(ctrl2), Some(ctrl3)).unwrap();
while let Err(e) = iv.reset(&mut Delay).await {
    Timer::after_secs(1).await;
}
    let mut lora = LoRa::new(Sx126x::new(spi, iv, config), false, Delay).await.unwrap();

    spawner.spawn(task(lora));
}
// `lora_phy::LoRa<Sx126x<SubghzSpiDevice<embassy_stm32::spi::Spi<'_, embassy_stm32::mode::Async>>, Stm32wlInterfaceVariant<embassy_stm32::gpio::Output<'_>>, Stm32wl>, embassy_time::Delay>`
#[embassy_executor::task]
pub async fn task(mut lora: LoRa<Sx126x<SubghzSpiDevice<Spi<'static,Async>>,Stm32wlInterfaceVariant<Output<'static>>, Stm32wl> ,Delay>) {

    Timer::after_secs(5).await;

    let mut receiving_buffer = [0u8; 256];


    let mdltn_params = {
        match lora.create_modulation_params(
            SpreadingFactor::_9,
            Bandwidth::_250KHz,
            CodingRate::_4_8,
            LORA_FREQUENCY_IN_HZ,
        ) {
            Ok(mp) => mp,
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }
    };

    let rx_pkt_params = {
        match lora.create_rx_packet_params(4, false, receiving_buffer.len() as u8, true, false, &mdltn_params) {
            Ok(pp) => pp,
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }
    };

    match lora
        .prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params)
        .await
    {
        Ok(()) => {}
        Err(err) => {
            info!("Radio error = {}", err);
            return;
        }
    };

    let mut rx_count = 0;
    loop {
        receiving_buffer = [0u8; 256];
        match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
            Ok((received_len, _rx_pkt_status)) => {
                if received_len > 2 && receiving_buffer[..3] == [36, 0, 20] {
                    info!("rx {} successful", rx_count);
                    rx_count+=1;
                } else {
                    warn!("unknown pkt: {}", &receiving_buffer[..received_len as _]);
                }
            }
            Err(err) => info!("rx unsuccessful = {}", err),
        }
    }
}
