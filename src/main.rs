#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m_semihosting::hprintln;
use embedded_graphics::{fonts::Font8x16, prelude::*};
use embedded_hal::digital::v2::OutputPin;
use rotary_encoder_hal::{Direction, Rotary};
use rtfm::app;
use rtfm::cyccnt::U32Ext;
use ssd1306::{interface::i2c::I2cInterface, prelude::*, Builder};
use stm32f1xx_hal::{
    gpio::gpiob::*,
    gpio::*,
    i2c::{BlockingI2c, DutyCycle, Mode},
    prelude::*,
    stm32::I2C1,
};

const ROTARY_ENCODER_PERIOD: u32 = 72_000;
const FLOW_COUNTER_PERIOD: u32 = 36_000;
const UPDATE_DISPLAY_PERIOD: u32 = 720_000;

static CONTAINER_SIZES: [(u16, &str); 7] = [
    (330, "330mL Bottle"),
    (500, "500mL Bottle"),
    (1000, "1L Bottle"),
    (2000, "2L Bottle"),
    (5000, "5L Keg"),
    (9540, "9.45L Keg"),
    (18000, "18L Keg"),
];

type Display = GraphicsMode<
    I2cInterface<BlockingI2c<I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>>,
>;

// We need to pass monotonic = rtfm::cyccnt::CYCCNT to use schedule feature fo RTFM
#[app(device = stm32f1xx_hal::pac, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        rotary_encoder: Rotary<PB10<Input<PullUp>>, PB11<Input<PullUp>>>,
        display: Display,
        container_choice: u8,
    }

    #[init(schedule = [scan_rotary_encoder, scan_flow_counter, update_display])]
    fn init(cx: init::Context) -> init::LateResources {
        // Enable cycle counter
        let mut core = cx.core;
        core.DWT.enable_cycle_counter();

        hprintln!("Init hardware...").unwrap();

        let device: stm32f1xx_hal::stm32::Peripherals = cx.device;

        // Setup clocks
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .pclk1(36.mhz())
            .freeze(&mut flash.acr);

        // Setup LED
        hprintln!("Setup LED on PC13").unwrap();
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);
        let mut led = gpioc
            .pc13
            .into_push_pull_output_with_state(&mut gpioc.crh, State::Low);
        led.set_low().unwrap();

        let mut portb = device.GPIOB.split(&mut rcc.apb2);
        //let mut _portc = device.GPIOC.split(&mut rcc.apb2);

        let mut _relay_pin = portb.pb1.into_push_pull_output(&mut portb.crl);

        hprintln!("Setting up rotary encoder").unwrap();
        let pin_a = portb.pb10.into_pull_up_input(&mut portb.crh);
        let pin_b = portb.pb11.into_pull_up_input(&mut portb.crh);

        let enc = Rotary::new(pin_a, pin_b);
        hprintln!("Setting up display").unwrap();
        let scl = portb.pb8.into_alternate_open_drain(&mut portb.crh);
        let sda = portb.pb9.into_alternate_open_drain(&mut portb.crh);

        let i2c = BlockingI2c::i2c1(
            device.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400_000.hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            &mut rcc.apb1,
            1000,
            10,
            1000,
            1000,
        );

        let mut disp: GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();
        hprintln!("Init display").unwrap();
        match disp.init() {
            Ok(_) => (),
            Err(e) => hprintln!("Failed to initialisde display {:?}", e).unwrap(),
        }
        match disp.display_on(true) {
            Ok(_) => (),
            Err(e) => hprintln!("Failed to call display on {:?}", e).unwrap(),
        }
        disp.clear();
        disp.flush().unwrap();

        let now = cx.start;

        hprintln!("Scheduling tasks").unwrap();
        // Schedule the blinking task
        cx.schedule
            .scan_rotary_encoder(now + ROTARY_ENCODER_PERIOD.cycles())
            .unwrap();
        cx.schedule
            .scan_flow_counter(now + FLOW_COUNTER_PERIOD.cycles())
            .unwrap();
        cx.schedule
            .update_display(now + UPDATE_DISPLAY_PERIOD.cycles())
            .unwrap();
        hprintln!("Init done").unwrap();
        led.set_high().unwrap();
        init::LateResources {
            rotary_encoder: enc,
            container_choice: 0,
            display: disp,
        }
    }

    #[task(schedule = [scan_rotary_encoder], resources = [rotary_encoder, container_choice])]
    fn scan_rotary_encoder(cx: scan_rotary_encoder::Context) {
        match cx.resources.rotary_encoder.update().unwrap() {
            Direction::Clockwise => {
                *cx.resources.container_choice += 1;
                if *cx.resources.container_choice >= (CONTAINER_SIZES.len() as u8) {
                    *cx.resources.container_choice = 0;
                }
            }
            Direction::CounterClockwise => {
                if *cx.resources.container_choice == 0 {
                    *cx.resources.container_choice = CONTAINER_SIZES.len() as u8;
                }
                *cx.resources.container_choice -= 1;
            }
            Direction::None => {}
        }

        cx.schedule
            .scan_rotary_encoder(cx.scheduled + ROTARY_ENCODER_PERIOD.cycles())
            .unwrap()
    }

    #[task(schedule = [scan_flow_counter])]
    fn scan_flow_counter(cx: scan_flow_counter::Context) {
        cx.schedule
            .scan_flow_counter(cx.scheduled + FLOW_COUNTER_PERIOD.cycles())
            .unwrap()
    }

    #[task(schedule = [update_display], resources = [container_choice, display])]
    fn update_display(cx: update_display::Context) {
        let display = cx.resources.display;
        let choice = *cx.resources.container_choice as usize;
        display.clear();
        display.draw(Font8x16::render_str(CONTAINER_SIZES[choice].1).into_iter());

        display.flush().unwrap();

        cx.schedule
            .update_display(cx.scheduled + UPDATE_DISPLAY_PERIOD.cycles())
            .unwrap();
    }

    extern "C" {
        fn EXTI0();
    }
};
