#![deny(unsafe_code)]
#![allow(clippy::missing_safety_doc)]
#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_semihosting::{hprintln};
use panic_semihosting as _;
use rtfm::cyccnt::{U32Ext as _};
//use stm32f1xx_hal::gpio::gpioc::*;
use stm32f1xx_hal::gpio::gpiob::*;
use stm32f1xx_hal::gpio::*;
use stm32f1xx_hal::prelude::*;

use rotary_encoder_hal::{Direction, Rotary};

const ROTARY_ENCODER_PERIOD: u32 = 720_000;
const FLOW_COUNTER_PERIOD: u32 = 500_000;

static CONTAINER_SIZES: [(u16, &str);7] = [
    (330, "330mL Bottle"),
    (500, "500mL Bottle"),
    (1000, "1L Bottle"),
    (2000, "2L Bottle"),
    (5000, "5L Keg"),
    (9540, "9.45L Keg"),
    (18000, "18L Keg"),
    ];


// RTFM main declaration
#[rtfm::app(device = stm32f1xx_hal::pac, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        rotary_encoder: Rotary<PB10<Input<PullUp>>,PB11<Input<PullUp>>>,
        container_choice: u8,
    }

    #[init(schedule = [scan_rotary_encoder, scan_flow_counter])]
    fn init(mut cx: init::Context) -> init::LateResources{

        cx.core.DCB.enable_trace();

        // semantically, the monotonic timer is frozen at time "zero" during `init`
        // NOTE do *not* call `Instant::now` in this context; it will return a nonsense value
        let now = cx.start; // the start time of the system

        // Device specific peripherals
        let device: stm32f1xx_hal::pac::Peripherals = cx.device;

        let mut rcc = device.RCC.constrain();
        let mut _afio = device.AFIO.constrain(&mut rcc.apb2);
        let mut flash = device.FLASH.constrain();
        let _clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .pclk1(36.mhz())
            .freeze(&mut flash.acr);

        let mut portb = device.GPIOB.split(&mut rcc.apb2);
        let mut portc = device.GPIOC.split(&mut rcc.apb2);

        let mut _led_pin = portc.pc13.into_push_pull_output(&mut portc.crh);
        let mut _relay_pin = portb.pb1.into_push_pull_output(&mut portb.crl);

        let pin_a = portb
            .pb10
            .into_pull_up_input(&mut portb.crh);
        let pin_b = portb
            .pb11
            .into_pull_up_input(&mut portb.crh);

        let enc = Rotary::new(pin_a, pin_b);

        hprintln!("init").unwrap();

        cx.schedule.scan_rotary_encoder(now + ROTARY_ENCODER_PERIOD.cycles()).unwrap();
        cx.schedule.scan_flow_counter(now + FLOW_COUNTER_PERIOD.cycles()).unwrap();

        init::LateResources{
            rotary_encoder: enc,
            container_choice: 0,
        }
    }

    #[task(schedule = [scan_rotary_encoder], resources = [rotary_encoder, container_choice])]
    fn scan_rotary_encoder(cx: scan_rotary_encoder::Context) {

        match cx.resources.rotary_encoder.update().unwrap() {
            Direction::Clockwise => {
                *cx.resources.container_choice+=1;
                if *cx.resources.container_choice >= (CONTAINER_SIZES.len() as u8) {
                    *cx.resources.container_choice = (CONTAINER_SIZES.len() as u8) - 1;
                }
            }
            Direction::CounterClockwise => {
                if *cx.resources.container_choice > 0 {
                    *cx.resources.container_choice-=1;
                }
            }
            Direction::None => {}
        }

        cx.schedule.scan_rotary_encoder(cx.scheduled + ROTARY_ENCODER_PERIOD.cycles()).unwrap()
    }

    #[task(schedule = [scan_flow_counter])]
    fn scan_flow_counter(cx: scan_flow_counter::Context) {

        cx.schedule.scan_flow_counter(cx.scheduled + FLOW_COUNTER_PERIOD.cycles()).unwrap()
    }

    extern "C" {
        fn USART1();
    }
};