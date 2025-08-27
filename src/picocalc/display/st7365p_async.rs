//! This file provides a ST7365P driver to connect to TFT displays.

use embassy_time::Instant;
use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::prelude::RawData;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};
use crate::debug;

#[derive(Debug, Clone, Copy)]
pub enum Instruction {
    NOP = 0x00,
    SWRESET = 0x01,
    RDDID = 0x04,
    RDDST = 0x09,
    SLPIN = 0x10,
    SLPOUT = 0x11,
    PTLON = 0x12,
    NORON = 0x13,
    INVOFF = 0x20,
    INVON = 0x21,
    DISPOFF = 0x28,
    DISPON = 0x29,
    CASET = 0x2A,
    RASET = 0x2B,
    RAMWR = 0x2C,
    RAMRD = 0x2E,
    PTLAR = 0x30,
    TEON = 0x35,
    COLMOD = 0x3A,
    MADCTL = 0x36,
    FRMCTR1 = 0xB1,
    FRMCTR2 = 0xB2,
    FRMCTR3 = 0xB3,
    DIC = 0xB4,
    ETMOD = 0xB7,
    MODESEL = 0xB9,
    PWR1 = 0xC0,
    PWR2 = 0xC1,
    PWR3 = 0xC2,
    VCMPCTL = 0xC5,
    RDID1 = 0xDA,
    RDID2 = 0xDB,
    RDID3 = 0xDC,
    PGC = 0xE0,
    NGC = 0xE1,
    DOCA = 0xE8,
    CSCON = 0xF0
}

/// ST7365P driver to connect to TFT displays.
pub struct ST7365P<SPI, DC, RST, DELAY>
where
    SPI: SpiDevice,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    /// SPI
    spi: SPI,

    /// Data/command pin.
    dc: DC,

    /// Reset pin.
    rst: Option<RST>,

    /// Whether the display is RGB (true) or BGR (false)
    rgb: bool,

    /// Whether the colours are inverted (true) or not (false)
    inverted: bool,

    /// Global image offset
    dx: u16,
    dy: u16,
    width: u32,
    height: u32,

    /// Delay
    delay: DELAY,
}

/// Display orientation.
#[derive(Clone, Copy)]
pub enum Orientation {
    Portrait = 0x00,
    Landscape = 0x60,
    PortraitSwapped = 0xC0,
    LandscapeSwapped = 0xA0,
}

impl<SPI, DC, RST, DELAY> ST7365P<SPI, DC, RST, DELAY>
where
    SPI: SpiDevice,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    /// Creates a new driver instance that uses hardware SPI.
    pub fn new(
        spi: SPI,
        dc: DC,
        rst: Option<RST>,
        rgb: bool,
        inverted: bool,
        width: u32,
        height: u32,
        delay: DELAY,
    ) -> Self {
        ST7365P {
            spi,
            dc,
            rst,
            rgb,
            inverted,
            dx: 0,
            dy: 0,
            width,
            height,
            delay,
        }
    }

    /// Runs commands to initialize the display.
    pub async fn init(&mut self) -> Result<(), ()> {
        self.hard_reset().await?;
        self.write_command(Instruction::SWRESET, &[]).await?;
        self.delay.delay_ms(200).await;
        self.write_command(Instruction::SLPOUT, &[]).await?;
        self.delay.delay_ms(200).await;

        self.write_command(Instruction::FRMCTR1, &[0x01, 0x2C, 0x2D])
            .await?;
        self.write_command(Instruction::FRMCTR2, &[0x01, 0x2C, 0x2D])
            .await?;
        self.write_command(Instruction::FRMCTR3, &[0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D])
            .await?;
        if self.inverted {
            self.write_command(Instruction::INVON, &[]).await?;
        } else {
            self.write_command(Instruction::INVOFF, &[]).await?;
        }
        if self.rgb {
            self.write_command(Instruction::MADCTL, &[0x00]).await?;
        } else {
            self.write_command(Instruction::MADCTL, &[0x08]).await?;
        }
        self.write_command(Instruction::COLMOD, &[0x05]).await?;

        Ok(())
    }

    /// Turns display on after init
    pub async fn set_on(&mut self) -> Result<(), ()>
    where
        DELAY: DelayNs,
    {
        self.write_command(Instruction::DISPON, &[]).await?;
        self.delay.delay_ms(200).await;
        Ok(())
    }

    pub async fn clear(&mut self, color: u16) -> Result<(), ()> {
        self.set_pixels_buffered(
            0,
            0,
            self.width as u16 - 1,
            self.height as u16 - 1,
            core::iter::repeat(RawU16::from(color).into_inner())
                .take((self.width * self.height) as usize),
        ).await
    }

    pub async fn hard_reset(&mut self) -> Result<(), ()>
    where
        DELAY: DelayNs,
    {
        if let Some(rst) = &mut self.rst {
            rst.set_high().map_err(|_| ())?;
            self.delay.delay_ms(10).await;
            rst.set_low().map_err(|_| ())?;
            self.delay.delay_ms(10).await;
            rst.set_high().map_err(|_| ())?;
        }
        Ok(())
    }

    async fn write_command(&mut self, command: Instruction, params: &[u8]) -> Result<(), ()> {
        // delay amount empirically determined
        self.delay.delay_ns(1).await;
        self.dc.set_low().map_err(|_| ())?;
        self.spi.write(&[command as u8]).await.map_err(|_| ())?;
        if !params.is_empty() {
            self.start_data()?;
            self.write_data(params).await?;
        }
        Ok(())
    }

    fn start_data(&mut self) -> Result<(), ()> {
        self.dc.set_high().map_err(|_| ())
    }

    async fn write_data(&mut self, data: &[u8]) -> Result<(), ()> {
        // delay amount empirically determined
        self.delay.delay_ns(1).await;
        self.spi.write(data).await.map_err(|_| ())
    }

    /// Writes a data word to the display.
    async fn write_word(&mut self, value: u16) -> Result<(), ()> {
        self.write_data(&value.to_be_bytes()).await
    }

    async fn write_words_buffered(&mut self, words: impl IntoIterator<Item = u16>) -> Result<(), ()> {
        //debug!("start: write_words_buffered");
        let mut buffer = [0_u8; 640];
        let mut index = 0;
        for word in words {
            let as_bytes = word.to_be_bytes();
            buffer[index] = as_bytes[0];
            buffer[index + 1] = as_bytes[1];
            index += 2;
            if index >= buffer.len() {
                self.write_data(&buffer).await?;
                index = 0;
            }
        }
        let result = self.write_data(&buffer[0..index]).await;
        //debug!("end: write_words_buffered");
        result
    }

    /// ensure you are only setting the top 3 bits for MADCTL (x,y, and x+y)
    pub async fn set_custom_orientation(&mut self, mut madctl: u8) -> Result<(), ()> {
        if !self.rgb {
            madctl |= 0x08
        }
        self.write_command(Instruction::MADCTL, &[madctl]).await?;

        Ok(())
    }

    pub async fn set_orientation(&mut self, orientation: &Orientation) -> Result<(), ()> {
        if self.rgb {
            self.write_command(Instruction::MADCTL, &[*orientation as u8])
                .await?;
        } else {
            self.write_command(Instruction::MADCTL, &[*orientation as u8 | 0x08])
                .await?;
        }
        Ok(())
    }

    /// Sets the global offset of the displayed image
    pub fn set_offset(&mut self, dx: u16, dy: u16) {
        self.dx = dx;
        self.dy = dy;
    }

    /// Sets the address window for the display.
    pub async fn set_address_window(&mut self, sx: u16, sy: u16, ex: u16, ey: u16) -> Result<(), ()> {
        //debug!("start: set_address_window");
        self.write_command(Instruction::CASET, &[]).await?;
        self.start_data()?;
        self.write_word(sx + self.dx).await?;
        self.write_word(ex + self.dx).await?;
        self.write_command(Instruction::RASET, &[]).await?;
        self.start_data()?;
        self.write_word(sy + self.dy).await?;
        let result = self.write_word(ey + self.dy).await;
        //debug!("end: set_address_window");
        result
    }

    /// Sets a pixel color at the given coords.
    pub async fn set_pixel(&mut self, x: u16, y: u16, color: u16) -> Result<(), ()> {
        self.set_address_window(x, y, x, y).await?;
        self.write_command(Instruction::RAMWR, &[]).await?;
        self.start_data()?;
        self.write_word(color).await
    }

    /// Writes pixel colors sequentially into the current drawing window
    pub async fn write_pixels<P: IntoIterator<Item = u16>>(&mut self, colors: P) -> Result<(), ()> {
        self.write_command(Instruction::RAMWR, &[]).await?;
        self.start_data()?;
        for color in colors {
            self.write_word(color).await?;
        }
        Ok(())
    }
    pub async fn write_pixels_buffered<P: IntoIterator<Item = u16>>(
        &mut self,
        colors: P,
    ) -> Result<(), ()> {
        //debug!("start: write_pixels_buffered");
        self.write_command(Instruction::RAMWR, &[]).await?;
        self.start_data()?;
        let result = self.write_words_buffered(colors).await;
        //debug!("end: write_pixels_buffered");
        result
    }

    /// Sets pixel colors at the given drawing window
    pub async fn set_pixels<P: IntoIterator<Item = u16>>(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
        colors: P,
    ) -> Result<(), ()> {
        self.set_address_window(sx, sy, ex, ey).await?;
        let result = self.write_pixels(colors).await;
        result
    }

    pub async fn set_pixels_buffered<P: IntoIterator<Item = u16>>(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
        colors: P,
    ) -> Result<(), ()> {
        //let instant = Instant::now();
        self.set_address_window(sx, sy, ex, ey).await?;
        //debug!("set_address_window: {}", instant.elapsed());
        let result = self.write_pixels_buffered(colors).await;
        result
    }

    /// Allows adjusting gamma correction on the display.
    ///
    /// Takes in an array `pos` for positive polarity correction and an array `neg` for negative polarity correction.
    ///
    pub async fn adjust_gamma(&mut self, pos: &[u8; 16], neg: &[u8; 16]) -> Result<(), ()> {
        self.write_command(Instruction::PGC, pos).await?;
        self.write_command(Instruction::NGC, neg).await
    }
}
