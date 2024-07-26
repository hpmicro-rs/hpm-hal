use super::*;
use crate::pac::sysctl::vals::AnaClkMux;
use crate::pac::SYSCTL;

impl_ana_clock_periph!(ADC0, ANA0, ADC0, adcclk, 0);
impl_ana_clock_periph!(ADC1, ANA1, ADC1, adcclk, 1);
impl_ana_clock_periph!(DAC0, ANA2, DAC0, dacclk, 0);
impl_ana_clock_periph!(DAC1, ANA3, DAC1, dacclk, 1);
