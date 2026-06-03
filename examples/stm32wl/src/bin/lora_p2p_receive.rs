//! This example runs on the STM32WL board, which has a builtin Semtech Sx1262 radio.
//! It demonstrates LORA P2P receive functionality in conjunction with the lora_p2p_send example.
#![no_std]
#![no_main]

#[path = "../iv.rs"]
mod iv;

use defmt::{warn, info, error};
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::spi::mode::Master;
use embassy_stm32::dma;
use embassy_time::{Delay, Timer};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{DMA1_CH1, DMA1_CH2, SUBGHZSPI};
use embassy_futures::{
    select::{Either, select},
    yield_now,
};
use lora_phy::sx126x::{Stm32wl, Sx126x, TcxoCtrlVoltage};
use lora_phy::mod_traits::InterfaceVariant;
use lora_phy::{mod_params::*, sx126x};
use lora_phy::{LoRa, RxMode, mod_traits::{IrqState, RadioKind}};
use {defmt_rtt as _, panic_probe as _};


use self::iv::{InterruptHandler, Stm32wlInterfaceVariant, SubghzSpiDevice};

const LORA_FREQUENCY_IN_HZ: u32 = 868_000_000; // warning: set this appropriately for the region

bind_interrupts!(struct Irqs{
    SUBGHZ_RADIO => InterruptHandler;
    DMA1_CHANNEL1 => dma::InterruptHandler<DMA1_CH1>;
    DMA1_CHANNEL2 =>  dma::InterruptHandler<DMA1_CH2>;
});

#[embassy_executor::main(
    executor = "embassy_stm32::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    #[cfg(feature = "lptim")]
    {
        use embassy_stm32::rcc::{*, mux::*};
        config.rcc.hse = Some(Hse {
            freq: Hertz(32_000_000),
            mode: HseMode::Bypass,
            prescaler: HsePrescaler::DIV1,
        });

        config.rcc.ls = LsConfig::default_lse();
        config.rcc.mux.lptim1sel = Lptimsel::LSE;
        config.rcc.mux.rngsel = Rngsel::PLL1_Q;
        config.rcc.mux.adcsel = Adcsel::SYS;
        config.rcc.ls.rtc = RtcClockSource::DISABLE;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL11,
            divp: Some(PllPDiv::DIV4),
            divq: Some(PllQDiv::DIV4), // PLL1_Q clock (32 / 2 * 6 / 2), used for RNG
            divr: Some(PllRDiv::DIV4), // sysclk 48Mhz clock (32 / 2 * 6 / 2)
        });
        config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_R;
        config.enable_debug_during_sleep = true;
    }
    let p = embassy_stm32::init(config);

    let ctrl1 = Output::new(p.PC4, Level::Low, Speed::High);
    let ctrl2 = Output::new(p.PC5, Level::Low, Speed::High);
    let ctrl3 = Output::new(p.PC3, Level::High, Speed::High);

    let spi = Spi::new_subghz(p.SUBGHZSPI, p.DMA1_CH1, p.DMA1_CH2, Irqs);
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

    spawner.spawn(task(lora).unwrap());
}
// `lora_phy::LoRa<Sx126x<SubghzSpiDevice<embassy_stm32::spi::Spi<'_, embassy_stm32::mode::Async>>, Stm32wlInterfaceVariant<embassy_stm32::gpio::Output<'_>>, Stm32wl>, embassy_time::Delay>`
#[embassy_executor::task]
pub async fn task(mut lora: LoRa<Sx126x<SubghzSpiDevice<Spi<'static,Async, Master>>,Stm32wlInterfaceVariant<Output<'static>>, Stm32wl> ,Delay>) {

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
    let selectable = true;
    if selectable {
        loop {
            match select(Timer::after_secs(15), wait_for_rx_irq(&mut lora, &mdltn_params, &rx_pkt_params)).await {
               Either::First(rx) =>  {
                    info!("timeout");
               }
                Either::Second(rx) => {
                     lora.clear_irq_status().await.unwrap();
                     let res = lora.get_rx_result(&rx_pkt_params, &mut receiving_buffer[..]).await;
                     match res {
            Ok((received_len, _rx_pkt_status)) => {
                            
                if received_len > 2 && receiving_buffer[..3] == [36, 0, 20] {
                    info!("rx {} successful", rx_count);
                    rx_count+=1;
                } else {
                    warn!("unknown pkt: {}", &receiving_buffer[..received_len as _]);
                }
                        }
                        Err(e) => {
                            error!("err");
                        }
                     }
                }
            }
        }
    } else {
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
    }}
}
// https://github.com/lora-rs/lora-rs/pull/379
/// Starts RX and waits for an RX IRQ. This can be safely canceled.
async fn wait_for_rx_irq<DEV: RadioKind, DLY: lora_phy::DelayNs>(
    radio: &mut LoRa<DEV, DLY>,
    mod_params: &ModulationParams,
    rx_params: &PacketParams,
) -> Result<(), RadioError> {
    radio.prepare_for_rx(RxMode::Continuous, mod_params, rx_params)
        .await?;
    radio.start_rx().await?;

    loop {
        radio.wait_for_irq().await?;
        match radio.get_irq_state().await {
            Ok(Some(IrqState::Done)) => {
                return Ok(());
            }
            Ok(_) => yield_now().await,
            Err(e) => return Err(e),
        }
    }
}
