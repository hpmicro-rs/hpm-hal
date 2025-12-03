use super::*;
use crate::pac::SYSCTL;
use crate::pac::sysctl::vals::AnaClkMux;

impl_ana_clock_periph!(ADC0, ANA0, ADC0, adcclk, 0);
